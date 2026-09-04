# CONCLUSION.md

Results and reproduction steps for the automatic aileron roll («керована бочка») in
ArduPlane. Companion to `CLAUDE.md` (settled design) and `PLAN.md` (order of work).

Everything below was measured, not estimated. Where a number is inferred rather than
directly observed, it says so.

---

## 1. What exists

A native C++ aerobatics state machine in `libraries/AP_Aerobatics/`, driven by a MAVLink
command, running **only in ACRO**:

```
IDLE → ENTRY → ROLLING → EXIT → IDLE
                  ↓
                ABORT → IDLE
```

- **Command**: `MAV_CMD_USER_1` (31010), so `modules/mavlink` stays untouched and stock
  MAVProxy/pymavlink work unmodified.
- **Hook**: `ModeAcro::stabilize()`, overriding the pilot rate targets.
- **Parameters**: `AEROB_RATE`, `AEROB_PITCH`, `AEROB_ALT_MIN`.
- **Aborts**: pilot stick, altitude floor, airspeed floor, ROLLING timeout, mode change.

Stages 1-6 complete, plus stage 7's code refactor. What remains of stage 7 is judgement,
not code: the tuning numbers are in §5.

---

## 2. Reproduce in SITL only — fastest, no Windows needed

This is the path for anyone who just wants to see it work. No RealFlight, no transmitter.

```bash
./waf configure --board sitl
./waf plane
sim_vehicle.py -v ArduPlane -f plane-3d --console --map -w
```

**Do not pass `--defaults Tools/autotest/default_params/plane.parm`** — that file does not
exist. `plane-3d` loads its own defaults from `@ROMFS/models/plane.parm`
([`SIM_Plane.cpp:105`](libraries/SITL/SIM_Plane.cpp#L105)).

In the MAVProxy console:

```
param set ARSPD_USE 1
param set AEROB_ALT_MIN 50
param set AEROB_RATE 180
param set AEROB_PITCH 25
param set TKOFF_ALT 120
param set SIM_SPEEDUP 3

mode TAKEOFF
arm throttle
```

Wait until altitude is above ~110 m, then:

```
mode ACRO
rc 3 1800
long 31010 1 1 1 0 0 0 0
```

Expected output:

```
AERO: ENTRY
AERO: roll right x1 at 180 deg/s
AERO: ROLLING
AERO: EXIT
AERO: done, roll=361 bank=5 pitch=3
AERO: IDLE
```

`roll=` is the integrated body-axis rotation; it should land within a few degrees of
`360 × reps`.

### Command format

```
long 31010 <maneuver> <direction> <reps> <rate> 0 0 0
```

| Param | Meaning |
|-------|---------|
| `param1` | maneuver: **1** = aileron roll (2 = loop, not implemented) |
| `param2` | direction: negative = left, otherwise right |
| `param3` | repetitions (1-5; 0 treated as 1) |
| `param4` | roll rate °/s (0 = use `AEROB_RATE`; clamped 30-360) |

### Result codes

| `COMMAND_ACK` | Meaning |
|---|---|
| 0 ACCEPTED | running |
| 1 TEMPORARILY_REJECTED | not in ACRO, or already running |
| 2 DENIED | disarmed |
| 3 UNSUPPORTED | unknown maneuver id |
| 4 FAILED | entry envelope rejected it — the reason is in the `AERO:` text |

### Or run the whole thing automatically

```bash
mkdir -p /tmp/aerosim && cd /tmp/aerosim
<repo>/build/sitl/bin/arduplane -S -I0 --model plane-3d --speedup 1 -w &
<repo>/Tools/autotest/aerobatics_sitl_test.py
```

It takes off, flies a clean roll, checks ENTRY/EXIT finish on attitude rather than their
time bounds, sweeps `*2SRV_TCONST` to prove the attitude gain tracks the plane's tuning,
provokes all five aborts, and checks both entry rejections. Exits 0 on success. Add
`--port` for a non-default instance.

### Provoking each abort by hand

Every one of these was verified in SITL. Trigger a 3-rep roll, then during ROLLING:

| Abort | How |
|---|---|
| pilot | `rc 1 1900` |
| altitude | `param set AEROB_ALT_MIN 1000` |
| airspeed | `param set AIRSPEED_MIN 60` |
| mode change | `mode FBWA` |
| timeout | `param set SERVO1_FUNCTION 0` (kills aileron; roll stops progressing) |

Restore each afterwards. Expect e.g.:

```
AERO: abort in ROLLING (altitude) roll=0
AERO: ABORT
AERO: recovered, bank=2 pitch=5
```

The pilot abort is deliberately different — it returns control on the same loop with **no**
recovery, because a stick input is a request for the aircraft, not for a recovery to fight.
Every other trigger flies the wings level first.

---

## 3. Reproduce in RealFlight + SITL — the one worth watching

### Windows side

RealFlight Evolution, everything under **Esc** (there is no menu bar):

- **Settings → Physics → Quality** — enable **RealFlight Link**
- **Settings → Physics** — *Pause Sim When in Background* = **No**,
  *Pauses Sim when in Menu* = **No**. Either left at Yes stalls SITL the moment you type in
  MAVProxy, which reads as a hang.
- **Settings → Physics** — *Automatic Reset Delay* = **2.0 s**
- Restart RealFlight after changing these; the Link server only starts at launch.
- Load the **Extra 300L**. FlightAxis returns only numeric telemetry — there is no aircraft
  name in the protocol, so nothing can verify this for you.
- Windows Firewall needs one inbound rule allowing TCP **18083** from the WSL subnet.

Transmitter plugs into **Windows** in USB Joystick (HID) mode, calibrated in RealFlight,
channel order AETR. It is not passed through to WSL.

### WSL side

```bash
./sim_realflight.sh -w
```

`-w` is required on the first run — the FlightAxis backend writes its own defaults to
EEPROM and they collide with leftovers from `plane-3d` runs. The script derives the Windows
host IP from the default route (WSL2 is NAT'd and the address changes on every Windows
reboot), and warns if 18083 is closed.

Check the link before anything else:

```bash
timeout 3 bash -c "cat < /dev/null > /dev/tcp/$(ip route show default | awk '{print $3}')/18083" \
  && echo reachable
```

### Parameters that must be set — see §6 for why

```
param set ARSPD_USE 1
param set RC2_REVERSED 0
param set SERVO2_REVERSED 1
param set AIRSPEED_MIN 20
param set AIRSPEED_CRUISE 38
```

### Flight mode switch

Check what your transmitter can actually reach before mapping modes. ArduPlane picks the
mode from PWM bands, so **you get as many modes as the switch has detents**:

| Position | PWM |
|---|---|
| 1 | < 1231 |
| 2 | 1231-1360 |
| 3 | 1361-1490 |
| 4 | 1491-1620 |
| 5 | 1621-1749 |
| 6 | ≥ 1750 |

A 2-position switch reaches only 1 and 6. A 3-position switch reaches 1, 5 and 6 (if its
middle detent is ~1642) or 1, 4 and 6 (if ~1500).

On the TX12 used here, the default `FLTMODE_CH 8` was 2-position — not enough for ACRO,
AUTOTUNE and MANUAL. Channels 6 and 7 were 3-position:

```
param set FLTMODE_CH 6
param set FLTMODE1 4     # ACRO
param set FLTMODE2 4     # ACRO
param set FLTMODE3 4     # ACRO
param set FLTMODE4 8     # AUTOTUNE
param set FLTMODE5 8     # AUTOTUNE
param set FLTMODE6 0     # MANUAL
```

Positions are **paired deliberately**. The middle detent measured 1642, only 21 µs above
the position 4/5 boundary at 1621 — too close to trust. Pairing 4 and 5 means it lands on
AUTOTUNE either way. There is a 200 ms debounce
([`RC_Channel.cpp:64`](libraries/RC_Channel/RC_Channel.cpp#L64)) so a brisk flick will not
catch intermediate bands, but a slow one can.

Verify your own switch rather than copying these numbers.

### AUTOTUNE first

Stock gains do not fly the Extra 300L well. Middle detent, then fly **large, sustained,
varied** roll and pitch inputs.

**Gains save per axis only if that axis found both a D limit and a P limit**
([`AP_AutoTune.cpp:142`](libraries/APM_Control/AP_AutoTune.cpp#L142)):

```c
if (is_positive(D_limit) && is_positive(P_limit)) {
    save_gains();
} else {
    restore_gains();   // everything that axis learned is thrown away
}
```

In this session roll saved and **pitch reverted** — so check your gains afterwards rather
than assuming. Roll after tuning:

| Param | Before | After |
|---|---|---|
| `RLL_RATE_P` | ~0.024 | **0.502** |
| `RLL_RATE_I` / `RLL_RATE_FF` | 0.024 / 0.291 | **0.289** |
| `RLL_RATE_D` | 0.005 | 0.008 |
| `RLL2SRV_RMAX` | 0 | **75** |

### Check the elevator before AUTOTUNE

```bash
Tools/autotest/aerobatics_check_pitch.py
```

Fly MANUAL above 15 m with clear held pitch inputs. It reports whether ArduPlane's internal
pitch loop is correctly signed. **Do this before AUTOTUNE** — §6.1 explains why a reversed
internal loop is invisible in MANUAL and diverges the moment you enter a stabilised mode.

### Fly the roll

Level, above ~80 m AGL, above ~28 m/s, ACRO, **hands off the sticks and throttle up**:

```
long 31010 1 1 1 0 0 0 0
```

To reproduce the sweep in §5:

```bash
Tools/autotest/aerobatics_sweep.py 120 180 240 300
```

It waits for a moment inside the entry envelope before each trigger rather than firing
blind, so a hand-flown aircraft wandering off level costs you a wait rather than a
rejection. Hands off throughout.

Sticks must stay centred — the pilot abort fires past 15% deflection, by design. Throttle
must stay **up**: it is never overridden, and idle throttle bleeds speed until the airspeed
abort triggers instead.

---

## 4. Other setups — hints

- **Any other FDM** (JSBSim, Gazebo, X-Plane): nothing in `AP_Aerobatics` is
  simulator-specific. It reads attitude through `AP::ahrs()` and takes altitude, airspeed,
  sticks and `dt` from the ACRO hook. Set `ARSPD_USE 1` and realistic `AIRSPEED_MIN` /
  `AIRSPEED_CRUISE` for whatever airframe you load, then §2 applies unchanged.
- **Real hardware**: untested and not recommended without a large safety margin. Raise
  `AEROB_ALT_MIN` well above the 50 m default, and fly the abort triggers deliberately
  before trusting the maneuver.
- **A different transmitter**: re-derive `RC2_REVERSED` and `SERVO2_REVERSED` (§6.1) rather
  than copying values. They are transmitter- and model-specific.
- **A different airframe**: re-run the rate sweep (§5). The saturation point is a property
  of the airframe and its tuning, not of this code.
- **Two MAVLink clients at once**: SITL exposes SERIAL1 on **5762** and SERIAL2 on **5763**
  alongside SERIAL0 on 5760. Useful for scripting on one port while MAVProxy holds another:
  `mavproxy.py --master tcp:127.0.0.1:5763`.
- **WSL clock skew**: "time moved backwards" warnings are cosmetic; `sudo hwclock -s`
  quiets them.

---

## 5. Results — the rate sweep

Extra 300L in RealFlight, roll tuned by AUTOTUNE, one repetition per rate, hands off.

| `AEROB_RATE` | Ideal 360/R | Measured ROLLING→done | Implied roll time | Tracking | Roll counted | Residual bank |
|---|---|---|---|---|---|---|
| 120 | 3.0 s | 3700 ms | ~3.0 s | **~100%** | 361° | 5° |
| 180 | 2.0 s | 2800 ms | ~2.1 s | **~95%** | 360° | 5° |
| 240 | 1.5 s | 2600 ms | ~1.9 s | **~79%** | 362° | 5° |
| 300 | 1.2 s | 2400 ms | ~1.7 s | **~71%** | 365° | 5° |

Measured times include the EXIT level-off; "implied roll time" subtracts a roughly constant
~0.7 s for it, so those figures are inferred rather than directly timed.

**Recommendation: `AEROB_RATE 180`.** Delivered rate saturates around 200 °/s however hard
you ask (roughly 120 → 171 → 189 → 212). 180 is the fastest demand the airframe still
honours; above it you gain little and just widen the gap between commanded and actual.

Three things this confirms:

- **The demand is not a guarantee.** At 300 °/s the roll still completes *correctly* —
  `roll_accumulated` integrates the true gyro rate — it just takes longer than
  `reps × 360 / rate` predicts. Sizing the ROLLING timeout at 3× ideal rather than from that
  arithmetic was necessary; a tight bound would abort good rolls.
- **The overshoot is discretisation, not error.** 365° at 300 °/s is one control loop:
  `300 °/s × 0.02 s = 6°`, and the termination check fires after the integration step. It
  scales with rate exactly as predicted (361° at 120, 365° at 300).
- **EXIT terminates on attitude, not its time bound.** Residual bank is 5° at every rate,
  which is precisely `AEROBATICS_EXIT_TOL`.

---

## 6. Findings — things that cost time, so they need not cost yours

### 6.1 The pitch reversal is a *pair*, and CLAUDE.md documents only half of it

`CLAUDE.md` says to set `RC2_REVERSED 0` for a TX12. True, but insufficient. The FlightAxis
default targets RealFlight's own InterLink controller
([`SIM_FlightAxis.cpp`](libraries/SITL/SIM_FlightAxis.cpp)):

```c
{ "RC2_REVERSED", 1 }, // interlink has reversed rc2
```

With `RC2_REVERSED=1` and `SERVO2_REVERSED=0`, two reversals cancel:

```
stick --[RC2_REVERSED]--> demand --[SERVO2_REVERSED]--> surface
```

MANUAL feels **correct** while ArduPlane's internal demand→surface chain is **backwards**.
That is the dangerous case: MANUAL flies fine, and every stabilised mode diverges in pitch,
because the stabiliser computes a correction and the surface moves the wrong way.

**Correct pair: `RC2_REVERSED 0`, `SERVO2_REVERSED 1`.** Flipping both preserves pilot feel
while fixing the internal sense.

**How to test it, and a trap to avoid.** Correlate ArduPlane's internal pitch demand
(RC-derived, with `RC2_REVERSED` applied) against measured pitch rate in MANUAL. Positive
agreement means correct.

Do **not** correlate `SERVO_OUTPUT_RAW` against pitch rate. That reading is taken *after*
`SERVO2_REVERSED`, so flipping the parameter flips both the number and the response — the
correlation is invariant and reads the same regardless of the setting. The general rule:
**a correlation can only detect a sign flip that sits outside the two signals being
correlated.** This cost an hour here.

### 6.2 Two more FlightAxis defaults are wrong

`sim_defaults[]` is plumbing only — no airframe tuning at all — and the `flightaxis:` frame
inherits the bare ArduPlane entry in `vehicleinfo.py`. Beyond the two `CLAUDE.md` lists:

| Param | Default | Measured for the Extra 300L |
|---|---|---|
| `AIRSPEED_MIN` | 9 m/s | floor **22.7** m/s → set **20** |
| `AIRSPEED_CRUISE` | 12 m/s | median **~40** m/s → set **38** |

`AIRSPEED_CRUISE` at 12 is below this aircraft's stall; TECS would try to fly it at under a
third of its real cruise. And at `AIRSPEED_MIN 9` the entry gate sits at 11.7 m/s against a
40 m/s cruise — it passes always, and the abort only fires once already stalled. **The
airspeed protection is nominal until this is set.**

Measured envelope over ~4,900 airborne samples: min 22.7, median 40.6, max 47.0 m/s.

### 6.3 AUTOTUNE moves `tau`, which is why the stage 7 refactor matters

`AUTOTUNE` writes the time constant
([`AP_AutoTune.cpp:506`](libraries/APM_Control/AP_AutoTune.cpp#L506)) and deliberately
targets a longer one on pitch ([`:572`](libraries/APM_Control/AP_AutoTune.cpp#L572)):

```c
if (type == AUTOTUNE_PITCH) {
    // 50% longer time constant on pitch
    target_tau *= 1.5;
}
```

Observed live: `PTCH2SRV_TCONST` went 0.5 → 0.75 on entering AUTOTUNE. The removed constant
`AEROBATICS_LEVEL_P = 2.0` equals 1/0.5, so it would have been **50% too aggressive on the
pitch axis** — and silently, since the maneuver still completes, just snatchier than the
airframe is tuned for. ENTRY and EXIT now read `tau` live and picked up 1/0.75 = 1.33
automatically.

(The pitch tune later reverted per §3, so the *stored* value is back at 0.5. The point
stands: AUTOTUNE demonstrably moves it.)

### 6.4 `*2SRV_RMAX` no longer defaults to 0 — `PLAN.md` is stale here

`PLAN.md` justifies keeping the library's own rate clamps by saying the matching
`*2SRV_RMAX` parameters default to 0 (no limit). **After an autotune they do not** —
`RLL2SRV_RMAX` became 75.

The conclusion still holds, more strongly: `_get_rate_out()` never applies RMAX (only
`get_servo_out()` does, and this code does not use that path), so
`AEROBATICS_PITCH_RATE_MAX` and `AEROBATICS_LEVEL_ROLL_RATE_MAX` remain the only clamps in
play. It is also a third argument against driving the angle controllers: a 75 °/s RMAX
would have capped ENTRY and EXIT.

### 6.5 The entry envelope earned its keep

During an unpowered vertical dive (dead battery, pitch **-87°**, descending 427 → 358 m),
four consecutive commands were refused:

```
AERO: not level, roll=-5 pitch=-87
REJECTED result=4
```

Worth noting **the airspeed gate would not have caught this** — 24-35 m/s was above the
26 m/s threshold. The attitude check did. That is a concrete argument for keeping all four
conditions rather than trimming any as redundant.

### 6.6 RealFlight shows SITL's output, not your transmitter

Control surfaces moving on their own is **not** a fault. Under FlightAxis, RealFlight
displays what ArduPilot commands. Measured while parked in RTL: transmitter channels 1 and 2
at spread **0**, aileron servo output at spread **505** — RTL trying to turn toward home
while sitting on the runway.

Only MANUAL shows your sticks alone. Any direction judgement made in a stabilised mode is
unreadable.

### 6.7 Enabling yaw rate control makes the rudder feel dead

`YAW_RATE_ENABLE=1` moves the rudder off the direct path onto a rate demand scaled by
`ACRO_YAW_RATE`. With `ACRO_YAW_RATE=30` and an untuned yaw PID, full stick asks for 30 °/s
and gets very little. Set both back to 0 for direct rudder.

---

## 7. Verified on the real airframe

Every hazard `CLAUDE.md` warns about has now been exercised in RealFlight, not only in SITL:

| Hazard | Evidence |
|---|---|
| `stabilize_quaternion()` short-circuit | Roll under `ACRO_LOCKING=2` + `ACRO_YAW_RATE=30` + `YAW_RATE_ENABLE=1` gave 2800 ms / 361° / 5° — identical to the unlocked run. Without the short-circuit the state machine would have frozen in ENTRY. |
| `acro_locking` bypass via explicit flag | Active during that same run |
| `gyro.x` integration, not Euler deltas | 360-365° across a 2.5× rate range, through inverted every time |
| `plane.G_Dt`, not hardcoded 0.02 | Overshoot scaled as `rate × dt` exactly |
| Entry envelope | §6.5 |
| Pilot abort | Fired on a real stick brush at 240 °/s |

**`YAW_RATE_ENABLE` must be 1 for the quaternion test to mean anything.** With it at 0,
`rate_control_enabled()` is false, that path is never a candidate, and the test is vacuous
whether or not the short-circuit works.

---

## 8. Scripts

Under `Tools/autotest/`, all three taking `--port` (default `tcp:127.0.0.1:5760`):

| Script | Purpose |
|---|---|
| `aerobatics_sitl_test.py` | Full automated suite in `plane-3d`. Takes off, flies a roll, checks the ENTRY/EXIT bounds and the `tau` coupling, provokes all five aborts, checks both entry rejections. Exit 0/1. |
| `aerobatics_sweep.py` | `AEROB_RATE` sweep against SITL or RealFlight. Assumes the aircraft is already flying in ACRO; waits for a valid entry window before each trigger. |
| `aerobatics_check_pitch.py` | Decides elevator direction from stick-vs-pitch-rate correlation in MANUAL. Its docstring explains the invariance trap in §6.1 that makes the obvious version of this test useless. |

SITL exposes SERIAL1 on **5762** and SERIAL2 on **5763**, so a script can hold one port
while MAVProxy holds another. A second instance (`-I1`) uses 5770.

## 9. Not done

- **The loop** (`param1 = 2`) is unimplemented; `start()` returns `BAD_MANEUVER` →
  `MAV_RESULT_UNSUPPORTED`.
- **Pitch is untuned** — AUTOTUNE reverted that axis. Roll, which the maneuver depends on,
  is tuned. Re-run with `AUTOTUNE_AXES 2` to work pitch alone.
- **Inverted ABORT recovery is untested in the air.** The `cos(roll)` scaling in
  `level_off()` is exercised and reasoned through, but every abort so far fired near
  upright. To test it deliberately: trigger a 3-rep roll and drop `AEROB_ALT_MIN` above
  current altitude at the halfway point of a slow roll.
- **No `AERO` dataflash message** — deferred until something needs a plot rather than a
  number. Standard `ATT` and `IMU` logging is on.
