# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

A fork of ArduPilot adding an **automatic aileron roll** to ArduPlane, triggered by a
MAVLink command and executable **only in ACRO mode**. A loop maneuver may follow if time
allows.

This is a course project. The deliverable is **working code**. Do not propose report
structure, results chapters, methodology justification, or "lessons learned" framing.
Slides are handled separately.

Terminology: *aileron roll*, not barrel roll — the latter follows a helical path.
Ukrainian: «бочка» / «керована бочка».

## Build and run

```bash
./waf configure --board sitl
./waf plane

# development: built-in FDM, fast iteration
sim_vehicle.py -v ArduPlane -f plane-3d --console --map

# watching the maneuver: RealFlight over FlightAxis (Windows host)
sim_vehicle.py -v ArduPlane -f flightaxis:<windows-ip> --console
```

Trigger the command from MAVProxy:

```
long 31000 <maneuver> <direction> <reps> <rate> 0 0 0
```

Wipe persisted params when experiments start behaving oddly: `sim_vehicle.py -w`.

## The MAVLink command

`MAV_CMD_USER_1` (31000) — a reserved user-defined slot in `common.xml`. Chosen so the
`modules/mavlink` submodule stays untouched and stock MAVProxy/pymavlink work unmodified.
Aliased in code:

```cpp
#define MAV_CMD_AEROBATIC_MANEUVER MAV_CMD_USER_1
```

| Param | Meaning |
|-------|---------|
| `param1` | maneuver ID: 1 = aileron roll, 2 = loop |
| `param2` | direction: -1 = left, +1 = right |
| `param3` | repetitions |
| `param4` | roll rate deg/s, 0 = use `AEROB_RATE` |

Result codes:

- `MAV_RESULT_TEMPORARILY_REJECTED` — not in ACRO, or a maneuver is already running
- `MAV_RESULT_DENIED` — disarmed
- `MAV_RESULT_FAILED` — entry envelope check failed

## Control logic

State machine `IDLE → ENTRY → ROLLING → EXIT → ABORT`, driven at 50 Hz from
`ModeAcro::update()`, overriding pilot rate targets while active.

**Abort unconditionally** on: pilot stick input, altitude floor, airspeed, timeout, or
mode change.

### Roll accumulation — the one thing not to get wrong

Integrate the body-axis roll rate:

```cpp
roll_accumulated += plane.ahrs.get_gyro().x * dt;
```

Do **not** accumulate deltas of `ahrs.roll`. Two independent failures:

1. It wraps at ±180°, so passing through inverted makes the counter jump backwards by
   nearly a full turn and the maneuver never terminates.
2. Euler roll rate carries a `tan(pitch)` term, which already corrupts the count at the
   ~25° entry pitch and is singular at 90° — the latter matters if the loop gets built.

`gyro.x` has no range limit, no singularity, and no cross-axis contamination. Gyro bias
drift over a 2–3 second maneuver is around a degree, which is irrelevant here.

### Entry envelope

Checked once, at command time; failure returns `MAV_RESULT_FAILED`.

- airspeed above roughly `1.3 × ARSPD_FBW_MIN`
- altitude AGL above `AEROB_ALT_MIN`
- roll and pitch both within ~20° of level
- armed and flying

The altitude and airspeed conditions are also checked continuously during the maneuver,
where they are abort triggers rather than rejections.

## Parameters

Own group `AEROB_`, not `ACRO_` (that prefix already belongs to ArduPlane's existing acro
params). MAVLink caps parameter names at **16 characters**.

| Name | Purpose | Units |
|------|---------|-------|
| `AEROB_RATE` | commanded body roll rate | deg/s |
| `AEROB_PITCH` | entry pitch-up target | deg |
| `AEROB_ALT_MIN` | altitude floor, AGL | m |

Keep it to these three — they exist so roll rate and entry pitch can be tuned without
rebuilding. Everything else stays a compile-time constant.

Never reuse a group index. Deleted parameters leave a gap; the index is how values are
located in EEPROM, and reusing one silently misreads stored data.

## Files

```
ArduPlane/GCS_Mavlink.cpp     command handler
ArduPlane/mode_acro.cpp       50 Hz hook
ArduPlane/mode.h
ArduPlane/Plane.h
ArduPlane/Parameters.cpp/.h   AEROB_ group registration
libraries/AP_Aerobatics/      state machine lives here
```

## Debugging

`gcs().send_text()` on each state transition, readable in MAVProxy while watching the
aircraft in RealFlight:

```cpp
gcs().send_text(MAV_SEVERITY_INFO, "AERO: %s roll=%.0f pitch=%.0f",
                state_name(), degrees(roll_accumulated), degrees(plane.ahrs.pitch));
```

A custom `AERO` dataflash message is deliberately deferred until something needs a plot
rather than a number. Standard `ATT` and `IMU` logging is already on and usually enough.

## Conventions

- ArduPilot style: 4 spaces, no tabs, `snake_case` members, `AP_` prefix on libraries.
- Commit messages prefixed with the subsystem: `ArduPlane: add aerobatics command handler`.
- Do not touch `modules/mavlink` or any other submodule.
- Keep commits small and buildable.

## Prior art

`libraries/AP_Scripting/applets/plane_aerobatics.lua` — existing Lua aerobatics running in
AUTO mode. Good reference for the maneuver math; this project is a native C++
implementation driven by MAVLink in ACRO.

## Environment

Ubuntu 24.04 / WSL2. These are settled — don't re-derive them:

- `python3-wxgtk4.0` from apt **before** pip, or wxPython builds from source for an hour
- `pip3 install --user --break-system-packages` (PEP 668 on 24.04)
- `empy` pinned to `3.3.4`; 4.x breaks waf
- `~/.local/bin` and `Tools/autotest` on `PATH`
- "time moved backwards" warnings are WSL clock skew; `sudo hwclock -s` quiets them and
  they are cosmetic

## Unreal Engine

Evaluated and dropped. Chaos provides rigid-body physics only — no lift, drag, or
propeller model — so using it would mean hand-writing a flight dynamics model. RealFlight
via FlightAxis covers the visual side with a real aerodynamic model already.
