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
./sim_realflight.sh
```

`sim_realflight.sh` resolves the Windows host IP from the default route and warns if the
FlightAxis port is closed. Don't hardcode the IP — see below.

Trigger the command from MAVProxy:

```
long 31010 <maneuver> <direction> <reps> <rate> 0 0 0
```

Wipe persisted params when experiments start behaving oddly: `sim_vehicle.py -w`.

## Simulation setup — done

**RealFlight Evolution over FlightAxis is set up, flown, and confirmed working.** Extra
300L, hand-flown in ACRO with a RadioMaster TX12. These are settled — don't re-derive
them, and don't re-litigate the simulator choice.

### Host link

WSL2 is NAT'd: the Windows host is the default gateway, and **the address changes on
every Windows reboot**. `sim_realflight.sh` derives it each run. SITL connects outbound to
TCP **18083**; Windows Firewall needs one inbound rule allowing that port from the WSL
subnet.

### RealFlight Evolution settings

Evolution has no top menu bar — everything is under **Esc**.

- **Settings → Physics → Quality** — enable **RealFlight Link**
- **Settings → Physics** — *Pause Sim When in Background* = **No**, *Pauses Sim when in
  Menu* = **No**. Either left at Yes stalls SITL the moment you type in MAVProxy, which
  reads as a hang.
- **Settings → Physics** — *Automatic Reset Delay (sec)* = **2.0**, so the final `AERO:`
  state text is readable before the sim snaps back to the runway.
- Restart RealFlight after changing these; the Link server only comes up on start.

### Transmitter

TX12 plugs into **Windows** in USB Joystick (HID) mode — not passed through to WSL.
RealFlight reads the sticks and forwards them to SITL as `rcin`
([`SIM_FlightAxis.cpp`](libraries/SITL/SIM_FlightAxis.cpp), 12 channels). Calibrate in
RealFlight, channel order AETR.

### Parameters the FlightAxis backend gets wrong

`sim_defaults[]` in `SIM_FlightAxis.cpp` is plumbing only (RC/servo ranges, EKF type,
accel offsets) — no airframe tuning at all. The `flightaxis:` frame inherits the bare
ArduPlane entry in `Tools/autotest/pysim/vehicleinfo.py`. Two fixes that are not optional:

- `RC2_REVERSED` is force-set to 1 for the RealFlight InterLink. Wrong for a TX12 —
  set it to **0** after verifying pitch direction with `rc guiin`. It persists to EEPROM.
- `ARSPD_USE` defaults to **0**, so airspeed is measured but unused. Set it to **1**.
  See the entry envelope section — the airspeed gate is meaningless without it.

Run `AUTOTUNE` (mode 8) before trusting ACRO rate tracking; stock gains do not fly the
Extra 300L well.

### Gotchas

- First FlightAxis run needs `-w` — the backend saves its own defaults to EEPROM and they
  collide with leftovers from `plane-3d` runs.

## The MAVLink command

`MAV_CMD_USER_1` (**31010**) — a reserved user-defined slot in `common.xml`. Chosen so the
`modules/mavlink` submodule stays untouched and stock MAVProxy/pymavlink work unmodified.
Aliased in code:

```cpp
#define MAV_CMD_AEROBATIC_MANEUVER MAV_CMD_USER_1
```

Not 31000 — that is `MAV_CMD_WAYPOINT_USER_1`, a mission command with `hasLocation="true"`.
The `MAV_CMD_USER_n` block starts at 31010, after five `WAYPOINT_USER_n` and five
`SPATIAL_USER_n` entries.

Implement the handler in `GCS_MAVLINK_Plane::handle_command_int_packet`. Handling
`command_int` alone covers both message types: `convert_COMMAND_LONG_to_COMMAND_INT` copies
`param1..param4` straight through, so a `command_long` from MAVProxy arrives intact.

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

State machine `IDLE → ENTRY → ROLLING → EXIT → ABORT`, overriding the pilot rate targets
in `ModeAcro::stabilize()` while active.

**Abort unconditionally** on: pilot stick input, altitude floor, airspeed, timeout, or
mode change.

### Where the hook goes

Not `ModeAcro::update()` — that only sets `nav_roll_cd` / `nav_pitch_cd` for reporting.
Control happens in `ModeAcro::run()` → `stabilize()`, where the pilot rate targets are
computed:

```cpp
float roll_rate  = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;
```

Replace both while active, then force the pure-rate branches. Two ways this breaks quietly:

- **Bypass `acro_locking` with an explicit flag.** The locking branches are guarded by
  `is_zero(roll_rate)`. ENTRY commands zero roll rate while pitching up, so relying on a
  non-zero rate to skip locking fails exactly when it matters.
- **Short-circuit `stabilize_quaternion()`.** With `ACRO_LOCKING=2` and non-zero
  `ACRO_YAW_RATE`, `run()` takes that path and never reaches `stabilize()`.

`update_control_mode` and `stabilize` are both `FAST_TASK`, so they run at
`SCHED_LOOP_RATE` — 50 Hz is only ArduPlane's default, and it is a parameter. Use
`plane.G_Dt` for the timestep, never a hardcoded `0.02`.

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

The airspeed gate requires `ARSPD_USE 1` in SITL — otherwise `airspeed_estimate()` returns
a throttle-and-attitude guess and the gate passes or fails for unrelated reasons.

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

Registered as a `ParametersG2` subgroup, reached as `plane.g2.aerobatics`:

```cpp
AP_SUBGROUPINFO(aerobatics, "AEROB_", 41, ParametersG2, AP_Aerobatics)
```

**41 is the next free G2 index** — the highest currently in use is 40 (`GUIDED_TIMEOUT`).

Never reuse a group index. Deleted parameters leave a gap; the index is how values are
located in EEPROM, and reusing one silently misreads stored data.

## Files

```
ArduPlane/GCS_MAVLink_Plane.cpp   command handler
ArduPlane/mode_acro.cpp           rate-target override in stabilize()
ArduPlane/mode.h
ArduPlane/Plane.h
ArduPlane/Parameters.cpp/.h       AEROB_ group registration
libraries/AP_Aerobatics/          state machine lives here
Tools/ardupilotwaf/ardupilotwaf.py  AP_Aerobatics in the library list
```

A library under `libraries/` cannot include `Plane.h`, and will not link until it is added
to `COMMON_VEHICLE_DEPENDENT_LIBRARIES`. Follow `AP_Quicktune`: a library ArduPlane already
uses, registered as a G2 subgroup, reaching vehicle state through the `AP::ahrs()` /
`AP::arming()` singletons. `AP_Aerobatics` reads attitude itself; the ACRO hook passes in
the plane-specific values — `dt`, altitude AGL, airspeed — and gets back desired rates.

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
