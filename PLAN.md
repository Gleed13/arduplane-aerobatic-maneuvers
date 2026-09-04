# PLAN.md

Implementation plan for the automatic aileron roll. Companion to `CLAUDE.md`, which holds
the settled design decisions; this file holds the order of work.

Status: simulation setup is done and flown. Stages 1-6 done; next is stage 7.

## Corrections — applied

Four errors found while reading the tree. **All four are now fixed in `CLAUDE.md`**, which
stays the source of truth; they are recorded here only so the reasoning is not lost.

**`MAV_CMD_USER_1` is 31010, not 31000.**

```
31000 → MAV_CMD_WAYPOINT_USER_1   hasLocation="true"
31010 → MAV_CMD_USER_1            hasLocation="false"
```

`modules/mavlink/message_definitions/v1.0/common.xml`. 31000 is a mission waypoint
command carrying a location, not a `command_long` slot. Every trigger example becomes:

```
long 31010 <maneuver> <direction> <reps> <rate> 0 0 0
```

**File names.** The command handler goes in `ArduPlane/GCS_MAVLink_Plane.cpp`, not
`GCS_Mavlink.cpp`. There is no `ArduPlane.cpp`; the scheduler table is in
`ArduPlane/Plane.cpp`.

**`ModeAcro::update()` is the wrong hook.** It only sets `nav_roll_cd` / `nav_pitch_cd`
for reporting and logging. Control happens in `ModeAcro::run()` → `ModeAcro::stabilize()`.

**Neither is a 50 Hz task.** `update_control_mode` and `stabilize` are both `FAST_TASK`
(`Plane.cpp`), so they run at `SCHED_LOOP_RATE`. 50 Hz is only ArduPlane's *default* — it
is a parameter. Use `plane.G_Dt` for the timestep, never a hardcoded `0.02`.

**A library under `libraries/` cannot include `Plane.h`,** and must be added to
`COMMON_VEHICLE_DEPENDENT_LIBRARIES` in `Tools/ardupilotwaf/ardupilotwaf.py` or it will
not link. See Architecture.

## Architecture

### Control hook

Override the pilot rate targets inside `ModeAcro::stabilize()`. The existing code computes:

```cpp
float roll_rate  = (rexpo/SERVO_MAX) * plane.g.acro_roll_rate;
float pitch_rate = (pexpo/SERVO_MAX) * plane.g.acro_pitch_rate;
```

Replace both while a maneuver is active, then force the pure-rate branches.

Two things that will silently break this:

- **Bypass `acro_locking` explicitly.** The locking branches are guarded by
  `is_zero(roll_rate)`. ENTRY commands *zero* roll rate while pitching up, so relying on a
  non-zero rate to skip locking fails exactly when it matters. Use an explicit active flag.
- **Short-circuit `stabilize_quaternion()`.** With `ACRO_LOCKING=2` and a non-zero
  `ACRO_YAW_RATE`, `run()` takes that path and never reaches `stabilize()`.

The alternative hook is the `nav_scripting_active()` branch in `ArduPlane/Attitude.cpp`,
which is how the Lua applet drives the aircraft. Not used here — it bypasses mode `run()`
wholesale and is not ACRO-specific, and this maneuver is ACRO-only by requirement. Read it
anyway: it is the reference for calling `rollController.get_rate_out()` and
`pitchController.get_rate_out()` directly.

### Library boundary

`AP_Quicktune` is the precedent: a library ArduPlane already uses, registered as a G2
subgroup, reaching vehicle state through `AP::ahrs()` / `AP::arming()` singletons, never
including `Plane.h`.

Split the same way:

- `AP_Aerobatics` reads attitude itself — gyro, roll, pitch — via `AP::ahrs()`
- the ACRO hook passes plane-specific values as arguments: `dt`, altitude AGL from
  `plane.relative_ground_altitude(...)`, airspeed
- it returns desired roll and pitch rates plus an active flag

Keeps the state machine unit-testable and honest about its inputs.

### Parameter registration

```cpp
AP_SUBGROUPINFO(aerobatics, "AEROB_", 41, ParametersG2, AP_Aerobatics)
```

**41 is the next free G2 index** — the highest currently in use is 40
(`GUIDED_TIMEOUT`). The object is a `ParametersG2` member, reached as
`plane.g2.aerobatics`.

## Stages

One small buildable commit each.

### 1 — Library skeleton and parameters

`libraries/AP_Aerobatics/` with the class, the three `AEROB_` params, the G2 subgroup at
index 41, and the waf library-list entry. State enum and `state_name()`, no logic.

Done when `param show AEROB_*` lists all three in SITL.

Isolated deliberately: the waf and G2 registration is the part with non-obvious failure
modes, and debugging it is much easier with no logic in the way.

### 2 — MAVLink command handler

Add `case MAV_CMD_AEROBATIC_MANEUVER:` to
`GCS_MAVLINK_Plane::handle_command_int_packet`.

Handling `command_int` alone covers both message types —
`convert_COMMAND_LONG_to_COMMAND_INT` copies `param1..param4` straight through, so a
`command_long` from MAVProxy arrives intact.

Implement only the three rejection paths and a `send_text` on accept:

| Result | Condition |
|--------|-----------|
| `MAV_RESULT_DENIED` | disarmed |
| `MAV_RESULT_TEMPORARILY_REJECTED` | not in ACRO, or already running |
| `MAV_RESULT_FAILED` | entry envelope (stage 3) |

Test every rejection from MAVProxy before writing any control code.

### 3 — Entry envelope

The four checks from `CLAUDE.md`. Set `ARSPD_USE 1` first or the airspeed gate is
meaningless. Confirm each check can be made to fail on demand.

### 4 — Roll accumulation and ROLLING

The core.

```cpp
roll_accumulated += AP::ahrs().get_gyro().x * dt;
```

`ArduPlane/mode_acro.cpp` already uses this exact idiom for roll locking
(`acro_state.locked_roll_err += ahrs.get_gyro().x * plane.G_Dt`) — same file, same
pattern, good precedent to point at in review.

Drive a constant `AEROB_RATE` until `|roll_accumulated| >= reps * 2π`, then go straight to
EXIT. No ENTRY pitch-up yet. First flyable version.

### 5 — ENTRY and EXIT

Pitch to `AEROB_PITCH` before rolling, level off after. State machine is now complete for
the roll.

### 6 — Aborts

Stick input, altitude floor, airspeed, timeout, mode change.

Mode change needs care: the maneuver object outlives the mode, so `ModeAcro::_enter()`
will not reset it for you. Reset explicitly on mode exit rather than assuming.

Two decisions worth keeping:

**The pilot gets the aircraft back immediately; everything else flies a recovery.** A
stick input is a request for the aircraft, so `update()` returns false on that loop and
the pilot's own `rexpo`/`pexpo` rates — already computed above the hook — take effect with
no gap. The other triggers go through `ABORT`, which levels both axes first: the pilot has
not asked for the aircraft and it may well be inverted.

**The level-off scales the pitch demand by `cos(roll)`.** The demand is a body rate, the
error is an Euler angle, and `theta_dot = q*cos(roll) - r*sin(roll)`. Unscaled, an abort
from the inverted half of a roll drives the nose *down* — exactly wrong for the altitude
trigger, which is the most likely reason to be there. Scaling reverses the elevator when
inverted and fades it out through knife edge. `EXIT` shares the same helper; near upright
`cos(roll)` is ~1, so it changes nothing there.

`VehicleState` is now read every loop rather than once at command time, and both callers
fill it through `Plane::get_aerobatics_state()` so the entry checks and the abort checks
cannot drift apart.

The `ROLLING` bound is sized in `start()` from what was actually commanded, at three times
`reps*360/rate`. See Main risk: the demand is not a guarantee, and a bound sized from that
arithmetic aborts good maneuvers.

Verified in SITL on `plane-3d`, all five triggers plus a clean roll and the two entry
rejections: `AERO: abort in ROLLING (pilot|altitude|airspeed|timeout|mode change)`.

### 7 — Tuning in RealFlight

`AUTOTUNE` first — stock gains do not fly the Extra 300L well and the rate controller must
track before any of the numbers mean anything. Then sweep `AEROB_RATE`.

**Take the level-off gain from the plane, not from a constant.** `AEROBATICS_LEVEL_P` (2.0)
duplicates ArduPlane's own attitude loop, which computes the same thing as
`angle_err_deg / gains.tau` in `AP_RollController.cpp` and `AP_PitchController.cpp`.
`gains.tau` is `RLL2SRV_TCONST` / `PTCH2SRV_TCONST`, default 0.5 s — so ArduPlane's default
is exactly 1/0.5 = 2.0, and the constant matches only by luck.

It matters here specifically: `AUTOTUNE` *adjusts* `tau`. Run it and the plane's attitude
loop retunes itself while ENTRY and EXIT go on using 2.0.

`gains` is `protected` in `AP_FW_Controller.h` with no accessor, so neither the library nor
the ACRO hook can read `tau` today. Two ways out:

- add a one-line `get_tau()` accessor and pass `1/tau` into `update()` — minimal, but
  modifies a shared upstream library for one consumer
- have ENTRY and EXIT drive the *angle* controllers instead of returning rates. Better
  shape, and the precedent is already in `mode_acro.cpp`: acro locking calls
  `rollController.get_servo_out(angle_err_cd, ...)` when the stick is centred and
  `get_rate_out(rate, ...)` when it is not. ENTRY and EXIT are attitude-holding phases and
  ROLLING is a rate phase, so the same split fits. Gets `tau`, `RMAX` and the full PID for
  free and deletes three constants.

Cost of the second: `Output` stops being two plain rates — it has to say whether each axis
is an angle or a rate demand, and the hook dispatches on that. Roughly 30 lines across the
library and the hook, plus a re-fly to confirm ENTRY and EXIT still finish on their
attitude conditions rather than their time bounds.

`AEROBATICS_PITCH_RATE_MAX` and `AEROBATICS_LEVEL_ROLL_RATE_MAX` are *not* redundant the
same way: the matching `*2SRV_RMAX` parameters default to 0, which means no limit.

### Stretch — loop

Only once the roll is solid. This is where the `tan(pitch)` singularity argument in
`CLAUDE.md` starts to matter in practice.

## Testing

`plane-3d` for logic — fast, no Windows in the loop. RealFlight from stage 4 on, where
seeing the maneuver is the point.

Per run: fly to altitude in `STABILIZE`, `mode ACRO`, trigger, watch the `AERO:` text in
MAVProxy against the aircraft in RealFlight.

## Main risk

**Rate-controller tracking.** `AEROB_RATE` is a demand, not a guarantee. If the tuning
cannot deliver it, `roll_accumulated` still integrates the true gyro rate — so the
maneuver completes correctly but takes longer than `reps * 360 / AEROB_RATE` predicts.

Size the timeout from measured behavior, not from that arithmetic, or stage 6 will produce
spurious aborts that look like a broken state machine.
