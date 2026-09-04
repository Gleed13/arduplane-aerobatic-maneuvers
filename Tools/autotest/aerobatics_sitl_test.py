#!/usr/bin/env python3
"""
Automated verification of the ArduPlane automatic aileron roll, in SITL.

Flies plane-3d from the runway and exercises everything: a clean roll, the
ENTRY/EXIT time bounds, the attitude gain tracking RLL2SRV_TCONST /
PTCH2SRV_TCONST, all five abort triggers, and both entry rejections.

Needs no RealFlight and no transmitter.

    ./waf configure --board sitl && ./waf plane
    mkdir -p /tmp/aerosim && cd /tmp/aerosim
    <repo>/build/sitl/bin/arduplane -S -I0 --model plane-3d --speedup 1 -w &
    Tools/autotest/aerobatics_sitl_test.py

Exits 0 if every check passes, 1 otherwise.

Note on threading: one reader thread owns the link. pymavlink connections are
not thread safe, and a second recv_match() steals the COMMAND_ACK the main
thread is waiting for. Status texts are stamped with the autopilot's own clock
so the durations below are sim time, independent of SIM_SPEEDUP.
"""
import argparse
import sys
import threading
import time

from pymavlink import mavutil

MODES = {'MANUAL': 0, 'STABILIZE': 2, 'ACRO': 4, 'FBWA': 5, 'FBWB': 6,
         'AUTO': 10, 'RTL': 11, 'LOITER': 12, 'TAKEOFF': 13}

MAV_CMD_AEROBATIC_MANEUVER = 31010

# entry bounds the library uses, mirrored here so the assertions are explicit
ENTRY_MS_MAX = 3000
EXIT_MS_MAX = 2000


class Link(object):
    """One reader thread, everything else reads the shared state it fills."""

    def __init__(self, port):
        self.m = mavutil.mavlink_connection(port, source_system=255)
        self.m.wait_heartbeat()
        self.lock = threading.Lock()
        self.texts = []          # (autopilot_ms, text)
        self.acks = []
        self.verbose = True
        # rc must exist before the override thread starts reading it
        self.rc = [1500, 1500, 1000, 1500]
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._rc, daemon=True).start()
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

    def _ap_ms(self):
        a = self.m.messages.get('ATTITUDE')
        return a.time_boot_ms if a else 0

    def _reader(self):
        while True:
            try:
                msg = self.m.recv_match(blocking=True, timeout=1)
            except Exception:
                continue
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == 'STATUSTEXT':
                with self.lock:
                    self.texts.append((self._ap_ms(), msg.text))
                if self.verbose:
                    print("   ST: %s" % msg.text, flush=True)
            elif kind == 'COMMAND_ACK':
                with self.lock:
                    self.acks.append(msg)

    def _rc(self):
        while True:
            self.m.mav.rc_channels_override_send(
                self.m.target_system, self.m.target_component,
                self.rc[0], self.rc[1], self.rc[2], self.rc[3], 0, 0, 0, 0)
            time.sleep(0.05)

    # ---- parameters ----------------------------------------------------
    def setp(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.4)

    # ---- marks and lookups ---------------------------------------------
    def mark(self):
        with self.lock:
            return len(self.texts), len(self.acks)

    def aero(self, mark):
        with self.lock:
            return [t for _, t in self.texts[mark[0]:] if t.startswith('AERO')]

    def stamp(self, mark, substr):
        """autopilot ms of the first AERO text after mark containing substr"""
        with self.lock:
            for ms, t in self.texts[mark[0]:]:
                if t.startswith('AERO') and substr in t:
                    return ms
        return None

    def duration(self, mark, first, second):
        a, b = self.stamp(mark, first), self.stamp(mark, second)
        return None if (a is None or b is None) else b - a

    def ack(self, mark, timeout=5):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if len(self.acks) > mark[1]:
                    return self.acks[mark[1]]
            time.sleep(0.05)
        return None

    def wait_aero(self, mark, substr, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if any(substr in t for t in self.aero(mark)):
                return True
            time.sleep(0.05)
        return False

    # ---- vehicle state --------------------------------------------------
    def mode(self, name):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            MODES[name], 0, 0, 0, 0, 0)
        end = time.time() + 10
        while time.time() < end:
            hb = self.m.messages.get('HEARTBEAT')
            if hb and hb.custom_mode == MODES[name]:
                return True
            time.sleep(0.1)
        print("!! mode %s not reached" % name, flush=True)
        return False

    def armed(self):
        hb = self.m.messages.get('HEARTBEAT')
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def agl(self):
        g = self.m.messages.get('GLOBAL_POSITION_INT')
        return g.relative_alt / 1000.0 if g else 0.0

    def airspeed(self):
        v = self.m.messages.get('VFR_HUD')
        return v.airspeed if v else 0.0

    def attitude(self):
        a = self.m.messages.get('ATTITUDE')
        return (0.0, 0.0) if a is None else (a.roll * 57.3, a.pitch * 57.3)

    def trigger(self, maneuver=1, direction=1, reps=1, rate=0):
        mark = self.mark()
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            MAV_CMD_AEROBATIC_MANEUVER, 0,
            maneuver, direction, reps, rate, 0, 0, 0)
        return mark, self.ack(mark)


class Results(object):
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append((name, bool(ok)))
        print(("PASS  " if ok else "FAIL  ") + name +
              (("   " + detail) if detail else ""), flush=True)

    def report(self):
        bad = [r for r in self.rows if not r[1]]
        print("\n===== SUMMARY =====", flush=True)
        for name, ok in self.rows:
            print(("PASS  " if ok else "FAIL  ") + name, flush=True)
        print("%d/%d passed" % (len(self.rows) - len(bad), len(self.rows)), flush=True)
        return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='tcp:127.0.0.1:5760')
    ap.add_argument('--speedup', type=float, default=3)
    ap.add_argument('--quiet', action='store_true', help='suppress status texts')
    args = ap.parse_args()

    link = Link(args.port)
    link.verbose = not args.quiet
    r = Results()
    print("connected, sysid %d" % link.m.target_system, flush=True)

    # ---------------------------------------------------------- setup
    print("\n== setup ==", flush=True)
    for name, value in [('ARMING_CHECK', 0), ('BRD_SAFETY_DEFLT', 0), ('ARSPD_USE', 1),
                        ('AEROB_ALT_MIN', 50), ('AEROB_RATE', 180), ('AEROB_PITCH', 25),
                        ('TKOFF_ALT', 120), ('THR_FAILSAFE', 0), ('FS_SHORT_ACTN', 0),
                        ('FS_LONG_ACTN', 0), ('SIM_SPEEDUP', args.speedup)]:
        link.setp(name, value)

    print("\n== takeoff ==", flush=True)
    link.mode('TAKEOFF')
    t0 = time.time()
    while time.time() - t0 < 25 and not link.armed():
        link.m.mav.command_long_send(
            link.m.target_system, link.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
    if not link.armed():
        print("FATAL: would not arm", flush=True)
        return 1
    link.rc[2] = 1800
    t0 = time.time()
    while time.time() - t0 < 120 and link.agl() < 110:
        time.sleep(0.5)
    print("alt agl %.0f  airspeed %.0f" % (link.agl(), link.airspeed()), flush=True)

    def settle(target=110):
        """climb back and level off between tests"""
        link.mode('FBWB')
        t0 = time.time()
        while time.time() - t0 < 120 and link.agl() < target:
            time.sleep(0.5)
        time.sleep(4)
        link.mode('ACRO')
        time.sleep(1)
        roll, pitch = link.attitude()
        print("   settled: alt %.0f  aspd %.0f  roll %.0f  pitch %.0f"
              % (link.agl(), link.airspeed(), roll, pitch), flush=True)

    settle()

    def fly_roll(reps=1, timeout=30):
        mark, ack = link.trigger(reps=reps)
        ok = ack is not None and ack.result == 0
        link.wait_aero(mark, 'done', timeout)
        return (mark, ok,
                link.duration(mark, 'ENTRY', 'ROLLING'),
                link.duration(mark, 'EXIT', 'done'))

    # ------------------------------------------------- 1: a clean roll
    print("\n== 1: normal roll completes ==", flush=True)
    mark, ok, t_entry, t_exit = fly_roll(1)
    r.check("accepted", ok)
    txt = link.aero(mark)
    r.check("reaches ROLLING", any('ROLLING' in t for t in txt))
    r.check("finishes normally", any('done' in t for t in txt), " | ".join(txt))
    r.check("no abort on a good roll", not any('abort' in t for t in txt))

    # ------------------------ 2: ENTRY/EXIT end on attitude, not timeout
    print("\n== 2: ENTRY and EXIT finish on attitude, not their time bounds ==", flush=True)
    print("   ENTRY %s ms (bound %d)   EXIT %s ms (bound %d)"
          % (t_entry, ENTRY_MS_MAX, t_exit, EXIT_MS_MAX), flush=True)
    r.check("ENTRY finishes on pitch, inside its bound",
            t_entry is not None and t_entry < ENTRY_MS_MAX, "%s ms" % t_entry)
    r.check("EXIT finishes on attitude, inside its bound",
            t_exit is not None and t_exit < EXIT_MS_MAX, "%s ms" % t_exit)

    # ------------------------------------ 3: the gain tracks *2SRV_TCONST
    print("\n== 3: attitude gain follows RLL2SRV_TCONST / PTCH2SRV_TCONST ==", flush=True)
    settle()
    link.setp('RLL2SRV_TCONST', 0.25)
    link.setp('PTCH2SRV_TCONST', 0.25)
    _, _, fast_entry, fast_exit = fly_roll(1)
    print("   tau 0.25 -> ENTRY %s ms  EXIT %s ms" % (fast_entry, fast_exit), flush=True)

    settle()
    link.setp('RLL2SRV_TCONST', 1.20)
    link.setp('PTCH2SRV_TCONST', 1.20)
    _, _, slow_entry, slow_exit = fly_roll(1)
    print("   tau 1.20 -> ENTRY %s ms  EXIT %s ms" % (slow_entry, slow_exit), flush=True)

    r.check("a larger tau slows the ENTRY pitch-up",
            None not in (fast_entry, slow_entry) and slow_entry > fast_entry,
            "0.25s: %s ms   1.20s: %s ms" % (fast_entry, slow_entry))
    r.check("a larger tau slows the EXIT level-off",
            None not in (fast_exit, slow_exit) and slow_exit > fast_exit,
            "0.25s: %s ms   1.20s: %s ms" % (fast_exit, slow_exit))
    link.setp('RLL2SRV_TCONST', 0.5)
    link.setp('PTCH2SRV_TCONST', 0.5)

    # ------------------------------------------------- 4: pilot abort
    print("\n== 4: pilot stick abort ==", flush=True)
    settle()
    mark, ack = link.trigger(reps=3)
    r.check("accepted (3 reps)", ack is not None and ack.result == 0)
    link.wait_aero(mark, 'ROLLING', 10)
    time.sleep(0.5)
    link.rc[0] = 1900
    time.sleep(1.5)
    link.rc[0] = 1500
    txt = link.aero(mark)
    r.check("pilot abort reported", any('(pilot)' in t for t in txt), " | ".join(txt))
    r.check("pilot abort flies no recovery", not any('recovered' in t for t in txt))

    # ---------------------------------------------- 5: altitude abort
    print("\n== 5: altitude floor abort ==", flush=True)
    settle()
    mark, ack = link.trigger(reps=3)
    r.check("accepted (3 reps)", ack is not None and ack.result == 0)
    link.wait_aero(mark, 'ROLLING', 10)
    link.setp('AEROB_ALT_MIN', 1000)
    link.wait_aero(mark, 'recovered', 15)
    txt = link.aero(mark)
    r.check("altitude abort reported", any('(altitude)' in t for t in txt), " | ".join(txt))
    r.check("altitude abort flies a recovery", any('recovered' in t for t in txt))
    link.setp('AEROB_ALT_MIN', 50)

    # ---------------------------------------------- 6: airspeed abort
    print("\n== 6: airspeed abort ==", flush=True)
    settle()
    mark, ack = link.trigger(reps=3)
    r.check("accepted (3 reps)", ack is not None and ack.result == 0)
    link.wait_aero(mark, 'ROLLING', 10)
    link.setp('AIRSPEED_MIN', 60)
    link.wait_aero(mark, 'recovered', 15)
    txt = link.aero(mark)
    r.check("airspeed abort reported", any('(airspeed)' in t for t in txt), " | ".join(txt))
    link.setp('AIRSPEED_MIN', 9)

    # ------------------------------------------- 7: mode change abort
    print("\n== 7: mode change abort ==", flush=True)
    settle()
    mark, ack = link.trigger(reps=3)
    r.check("accepted (3 reps)", ack is not None and ack.result == 0)
    link.wait_aero(mark, 'ROLLING', 10)
    time.sleep(0.5)
    link.mode('FBWA')
    time.sleep(2)
    txt = link.aero(mark)
    r.check("mode change abort reported",
            any('(mode change)' in t for t in txt), " | ".join(txt))

    # ----------------------------------------------- 8: ROLLING timeout
    print("\n== 8: ROLLING timeout ==", flush=True)
    settle()
    mark, ack = link.trigger(reps=1)
    r.check("accepted", ack is not None and ack.result == 0)
    link.wait_aero(mark, 'ROLLING', 10)
    link.setp('SERVO1_FUNCTION', 0)      # aileron gone; the roll cannot progress
    link.wait_aero(mark, '(timeout)', 25)
    txt = link.aero(mark)
    r.check("timeout abort reported", any('(timeout)' in t for t in txt), " | ".join(txt))
    link.setp('SERVO1_FUNCTION', 4)

    # ------------------------------------ 9: entry rejections unchanged
    print("\n== 9: entry rejections ==", flush=True)
    settle()
    link.setp('AEROB_ALT_MIN', 1000)
    mark, ack = link.trigger(reps=1)
    r.check("envelope rejection is MAV_RESULT_FAILED (4)",
            ack is not None and ack.result == 4,
            "result=%s" % (ack.result if ack else None))
    link.setp('AEROB_ALT_MIN', 50)
    link.mode('FBWA')
    mark, ack = link.trigger(reps=1)
    r.check("non-ACRO is TEMPORARILY_REJECTED (1)",
            ack is not None and ack.result == 1,
            "result=%s" % (ack.result if ack else None))

    return r.report()


if __name__ == '__main__':
    sys.exit(main())
