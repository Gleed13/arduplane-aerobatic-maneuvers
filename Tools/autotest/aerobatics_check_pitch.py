#!/usr/bin/env python3
"""
Decide elevator direction from data, not from convention.

Correlates ArduPlane's INTERNAL pitch demand against the aircraft's measured
pitch rate while flying in MANUAL. Positive agreement means the internal loop
is correctly signed; negative means SERVO2_REVERSED needs flipping.

    Tools/autotest/aerobatics_check_pitch.py
    Tools/autotest/aerobatics_check_pitch.py --port tcp:127.0.0.1:5762 --seconds 60

Fly in MANUAL, above 15 m, with clear held nose-up / nose-down inputs.

WHY THIS TEST AND NOT AN EASIER ONE
-----------------------------------
The obvious test -- correlate SERVO_OUTPUT_RAW against pitch rate -- does not
work. That value is reported AFTER SERVO2_REVERSED is applied, so flipping the
parameter flips both the number you measure and the aircraft's response. The
correlation is invariant and reads the same regardless of the setting.

The general rule: a correlation can only detect a sign flip that sits OUTSIDE
the two signals being correlated.

  control_in  depends on RC2_REVERSED, not on SERVO2_REVERSED
  pitch rate  depends on both
  => correlating them detects SERVO2_REVERSED, and is blind to RC2_REVERSED

So this script answers "is the internal demand->response chain correctly
signed?", which is what every stabilised mode depends on. It deliberately does
NOT answer "does pulling back raise the nose?" -- that needs one bit of
information no telemetry carries, namely which way the pilot physically moved
the stick.

THE TRAP THIS EXISTS TO CATCH
-----------------------------
RC2_REVERSED and SERVO2_REVERSED can be wrong in a compensating pair:

    stick --[RC2_REVERSED]--> demand --[SERVO2_REVERSED]--> surface

With 1/0 the two reversals cancel, so MANUAL feels perfectly correct while the
internal chain is backwards. MANUAL flies fine and every stabilised mode
diverges in pitch. On the RealFlight Extra 300L the correct pair is
RC2_REVERSED 0, SERVO2_REVERSED 1.
"""
import argparse
import sys
import threading
import time

from pymavlink import mavutil

MANUAL = 0
MIN_AGL_M = 15
STICK_THRESHOLD = 0.20      # only sample while a real input is held


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='tcp:127.0.0.1:5760')
    ap.add_argument('--seconds', type=float, default=45)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port, source_system=249)
    m.wait_heartbeat()
    params = {}

    def reader():
        while True:
            try:
                msg = m.recv_match(blocking=True, timeout=1)
            except Exception:
                continue
            if msg is not None and msg.get_type() == 'PARAM_VALUE':
                params[msg.param_id.strip('\x00')] = msg.param_value

    threading.Thread(target=reader, daemon=True).start()
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 25, 1)

    for name in ('RC2_REVERSED', 'SERVO2_REVERSED'):
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      name.encode(), -1)
        time.sleep(0.3)
    time.sleep(1.5)

    rev = int(params.get('RC2_REVERSED', 0))
    servo_rev = params.get('SERVO2_REVERSED')
    print("RC2_REVERSED = %s   SERVO2_REVERSED = %s" % (rev, servo_rev), flush=True)
    print("fly MANUAL with clear pitch inputs -- sampling %gs" % args.seconds, flush=True)

    demands, rates = [], []
    end = time.time() + args.seconds
    while time.time() < end:
        hb = m.messages.get('HEARTBEAT')
        rc = m.messages.get('RC_CHANNELS')
        att = m.messages.get('ATTITUDE')
        gpi = m.messages.get('GLOBAL_POSITION_INT')
        if hb and rc and att and gpi:
            if hb.custom_mode == MANUAL and gpi.relative_alt / 1000.0 > MIN_AGL_M:
                raw = rc.chan2_raw
                if raw not in (0, 65535):
                    demand = (raw - 1500) / 500.0
                    if rev:
                        demand = -demand           # ArduPlane's own convention
                    if abs(demand) > STICK_THRESHOLD:
                        demands.append(demand)
                        rates.append(att.pitchspeed)
        time.sleep(0.04)

    if len(demands) < 25:
        print("VERDICT not enough data (%d samples) -- need MANUAL, airborne, "
              "and real elevator input" % len(demands), flush=True)
        return 2

    agree = sum(1 for d, q in zip(demands, rates) if d * q > 0) / len(demands)
    print("DEMAND n=%d  sign agreement %.0f%%" % (len(demands), agree * 100), flush=True)

    if agree > 0.7:
        print("VERDICT internal pitch loop is CORRECTLY SIGNED", flush=True)
        return 0
    if agree < 0.3:
        want = 0 if (servo_rev and servo_rev > 0.5) else 1
        print("VERDICT internal pitch loop is REVERSED -- set SERVO2_REVERSED=%d" % want,
              flush=True)
        print("        (and flip RC2_REVERSED too if that changes how the stick feels;"
              "\n         the pair must both move to keep pilot feel unchanged)", flush=True)
        return 1
    print("VERDICT inconclusive -- need larger, cleaner pitch inputs", flush=True)
    return 2


if __name__ == '__main__':
    sys.exit(main())
