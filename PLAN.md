# PLAN.md

Implementation plan for the automatic aileron roll. Companion to `CLAUDE.md`, which holds
the settled design decisions; this file holds the order of work.

Status: simulation setup is done and flown. Stages 1-6 done. Stage 7's code refactor is
done; what remains of stage 7 is the RealFlight tuning itself — AUTOTUNE, then the
`AEROB_RATE` sweep.

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

#### The gain refactor — done

**Take the level-off gain from the plane, not from a constant.** `AEROBATICS_LEVEL_P` (2.0)
duplicated ArduPlane's own attitude loop, which computes the same thing as
`angle_err_deg / gains.tau` in `AP_RollController.cpp` and `AP_PitchController.cpp`.
`gains.tau` is `RLL2SRV_TCONST` / `PTCH2SRV_TCONST`, default 0.5 s — so ArduPlane's default
is exactly 1/0.5 = 2.0, and the constant matched only by luck.

It matters here specifically: `AUTOTUNE` *adjusts* `tau`. Run it and the plane's attitude
loop retunes itself while ENTRY and EXIT go on using 2.0.

**Correction to this plan.** The two options below were written against a wrong reading of
the tree, and the reasoning that picked between them does not survive checking:

- `gains` is protected, but **`AP_FW_Controller` already exposes `AP_Float &tau(void)`**,
  public. No accessor needs adding and no upstream file is touched — so the stated cost of
  the first option ("modifies a shared upstream library for one consumer") is zero.
  `ModeAcro::stabilize_quaternion()` already reads it exactly this way
  (`desired_rates.x /= plane.rollController.tau()`), in the same file as our hook.
- The second option carries a hazard this plan did not anticipate.
  `AP_PitchController::get_servo_out()` does not just divide by `tau`; it adds
  `_get_coordination_rate_offset()`, a turn-coordination pitch-up proportional to
  `tan(bank)·sin(bank)`. **An aileron roll is not a turn.** The term is bounded (±80°
  upright, 100–180° inverted) but is already large at 80° of bank, and `ABORT` can begin at
  *any* bank angle. Driving the angle controller would import a turn-coordination pull
  straight into the inverted recovery.

So the first option was taken. `VehicleState` carries `roll_tau` and `pitch_tau`, filled
live by `Plane::get_aerobatics_state()`, and `angle_to_rate()` divides by them with the
same 0.05 s floor `AP_FW_Controller` applies.

Only **one** constant went, not three. `AEROBATICS_PITCH_RATE_MAX` and
`AEROBATICS_LEVEL_ROLL_RATE_MAX` stay — the matching `*2SRV_RMAX` parameters default to 0,
which means no limit — and the `cos(roll)` scaling in `level_off()` stays, since it is
doing a different job from `tau`.

Verified in SITL by sweeping the parameter and timing the states:

| `*2SRV_TCONST` | ENTRY | EXIT |
|---|---|---|
| 0.25 s | 1000 ms | 0 ms |
| 0.50 s | 1000 ms | 1000 ms |
| 1.20 s | 2000 ms | 2000 ms (its bound) |

Two things that table says, both worth carrying into the tuning session:

- **ENTRY is rate-limited, not tau-limited, at 0.5 s and below.** A ~29° pitch error over
  `tau` 0.5 gives 58 °/s, above the 45 °/s `AEROBATICS_PITCH_RATE_MAX` clamp, so the clamp
  sets the duration. `tau` only starts driving ENTRY once it is large.
- **At `tau` 1.20 s, EXIT stops finishing on attitude and falls back to its 2000 ms bound.**
  That is correct behaviour, not a bug — but if `AUTOTUNE` lands on a large `tau`, expect
  `AERO: done` to start reporting a few degrees of residual bank, and raise
  `AEROBATICS_EXIT_MS_MAX` rather than reintroducing a fixed gain.

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
