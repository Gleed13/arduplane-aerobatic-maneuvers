#!/usr/bin/env python3
"""
Sweep AEROB_RATE and record how each aileron roll actually went.

Works against plain SITL or SITL-over-FlightAxis. It does not take off or
change mode: the aircraft must already be flying in ACRO. The pilot keeps
hands off -- any stick input aborts the maneuver by design.

    Tools/autotest/aerobatics_sweep.py                    # 120 180 240 300
    Tools/autotest/aerobatics_sweep.py 180
    Tools/autotest/aerobatics_sweep.py --port tcp:127.0.0.1:5762 200 250

Before each trigger it waits for a moment inside the entry envelope, with
margin, rather than firing blind and collecting rejections -- a hand-flown
aircraft wanders, and the envelope check is strict about attitude.

Reading the results: the reported duration runs ROLLING -> done, so it
INCLUDES the EXIT level-off (roughly 0.7 s on the Extra 300L). Subtract that
before comparing against the ideal 360/rate. AEROB_RATE is a demand, not a
guarantee: if the tuning cannot deliver it the roll still completes correctly
-- roll_accumulated integrates the true gyro rate -- it just takes longer.
"""
import argparse
import sys
import threading
import time

from pymavlink import mavutil

MAV_CMD_AEROBATIC_MANEUVER = 31010
ACRO = 4

# entry window, with margin over what the library actually enforces
MIN_AGL_M = 80
MIN_AIRSPEED = 28
MAX_TILT_DEG = 15


class Link(object):
    def __init__(self, port):
        self.m = mavutil.mavlink_connection(port, source_system=250)
        self.m.wait_heartbeat()
        self.lock = threading.Lock()
        self.texts = []
        self.acks = []
        threading.Thread(target=self._reader, daemon=True).start()
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
                if msg.text.startswith('AERO'):
                    print("   >> %s" % msg.text, flush=True)
            elif kind == 'COMMAND_ACK':
                with self.lock:
                    self.acks.append(msg)

    def setp(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                  name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.6)

    def mark(self):
        with self.lock:
            return len(self.texts), len(self.acks)

    def aero(self, mark):
        with self.lock:
            return [t for _, t in self.texts[mark[0]:] if t.startswith('AERO')]

    def stamp(self, mark, substr):
        with self.lock:
            for ms, t in self.texts[mark[0]:]:
                if t.startswith('AERO') and substr in t:
                    return ms
        return None

    def ack(self, mark, timeout=5):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if len(self.acks) > mark[1]:
                    return self.acks[mark[1]]
            time.sleep(0.05)
        return None

    def armed(self):
        hb = self.m.messages.get('HEARTBEAT')
        return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def in_acro(self):
        hb = self.m.messages.get('HEARTBEAT')
        return bool(hb and hb.custom_mode == ACRO)

    def agl(self):
        g = self.m.messages.get('GLOBAL_POSITION_INT')
        return g.relative_alt / 1000.0 if g else 0.0

    def airspeed(self):
        v = self.m.messages.get('VFR_HUD')
        return v.airspeed if v else 0.0

    def attitude(self):
        a = self.m.messages.get('ATTITUDE')
        return (0.0, 0.0) if a is None else (a.roll * 57.3, a.pitch * 57.3)


def wait_window(link, timeout=120):
    """wait for a moment satisfying the entry envelope, with margin"""
    start = time.time()
    told = 0.0
    while time.time() - start < timeout:
        roll, pitch = link.attitude()
        ok_mode = link.in_acro()
        ok_att = abs(roll) < MAX_TILT_DEG and abs(pitch) < MAX_TILT_DEG
        ok_alt = link.agl() > MIN_AGL_M
        ok_spd = link.airspeed() > MIN_AIRSPEED
        if ok_mode and ok_att and ok_alt and ok_spd and link.armed():
            return True
        if time.time() - told > 10:
            told = time.time()
            print("   waiting: acro=%s att=%s alt=%s spd=%s"
                  "  (roll %.0f pitch %.0f alt %.0f spd %.1f)"
                  % (ok_mode, ok_att, ok_alt, ok_spd,
                     roll, pitch, link.agl(), link.airspeed()), flush=True)
        time.sleep(0.2)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='tcp:127.0.0.1:5760')
    ap.add_argument('--reps', type=int, default=1)
    ap.add_argument('--settle', type=float, default=12,
                    help='seconds between rolls (default 12)')
    ap.add_argument('rates', nargs='*', type=float,
                    default=[120, 180, 240, 300])
    args = ap.parse_args()

    link = Link(args.port)
    print("connected, sysid %d" % link.m.target_system, flush=True)

    rows = []
    for rate in args.rates:
        print("\n=== AEROB_RATE %g ===" % rate, flush=True)
        if not wait_window(link):
            print("   NO WINDOW - skipped", flush=True)
            rows.append((rate, 'no window', None, None, ''))
            continue

        roll, pitch = link.attitude()
        print("   pre-roll: alt %.0f m  aspd %.1f m/s  roll %.0f  pitch %.0f"
              % (link.agl(), link.airspeed(), roll, pitch), flush=True)

        link.setp('AEROB_RATE', rate)
        mark = link.mark()
        link.m.mav.command_long_send(
            link.m.target_system, link.m.target_component,
            MAV_CMD_AEROBATIC_MANEUVER, 0, 1, 1, args.reps, 0, 0, 0, 0)
        ack = link.ack(mark)
        if ack is None or ack.result != 0:
            print("   REJECTED result=%s" % (None if ack is None else ack.result), flush=True)
            rows.append((rate, 'rejected', None, None,
                         " | ".join(link.aero(mark))))
            continue

        end = time.time() + 30
        while time.time() < end:
            txt = link.aero(mark)
            if any(('done' in t or 'recovered' in t) for t in txt):
                break
            time.sleep(0.1)

        txt = link.aero(mark)
        t_roll = link.stamp(mark, 'ROLLING')
        t_end = link.stamp(mark, 'done') or link.stamp(mark, 'recovered')
        dur = (t_end - t_roll) if (t_roll is not None and t_end is not None) else None
        done = next((t for t in txt if 'done' in t), '')
        aborted = next((t for t in txt if 'abort' in t), '')

        ideal_ms = args.reps * 360.0 / rate * 1000.0
        rows.append((rate, 'abort' if aborted else 'ok', dur, ideal_ms,
                     done or aborted))
        print("   ROLLING->done %s ms (ideal roll %.0f ms)   %s"
              % (dur, ideal_ms, done or aborted), flush=True)
        print("   settling %gs" % args.settle, flush=True)
        time.sleep(args.settle)

    print("\n===== SWEEP =====", flush=True)
    print("%-8s %-10s %-12s %-10s %s"
          % ("rate", "result", "ROLL->done", "ideal", "text"), flush=True)
    for rate, result, dur, ideal, text in rows:
        print("%-8g %-10s %-12s %-10s %s"
              % (rate, result,
                 "-" if dur is None else "%d ms" % dur,
                 "-" if ideal is None else "%.0f ms" % ideal,
                 text), flush=True)
    print("\nDurations include the EXIT level-off; subtract it (~700 ms on the "
          "Extra 300L)\nbefore comparing against 'ideal'.", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
