#pragma once

#include "AP_Aerobatics_config.h"

#if AP_AEROBATICS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/*
  MAV_CMD_USER_1 (31010) is a reserved user-defined slot in common.xml,
  chosen so the mavlink submodule stays untouched and stock
  MAVProxy/pymavlink work unmodified. Not 31000 -- that is
  MAV_CMD_WAYPOINT_USER_1, a mission command carrying a location.
 */
#define MAV_CMD_AEROBATIC_MANEUVER MAV_CMD_USER_1

/*
  automatic aerobatic maneuvers for ArduPlane, triggered by
  MAV_CMD_AEROBATIC_MANEUVER and run only in ACRO mode.

  This holds the maneuver state machine. It reads attitude through the
  AP::ahrs() singleton; everything vehicle specific (timestep, altitude
  AGL, airspeed) is passed in by the ACRO hook.
 */
class AP_Aerobatics {
public:
    AP_Aerobatics()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Aerobatics);

    static const struct AP_Param::GroupInfo var_info[];

    // maneuver ID, as sent in param1 of MAV_CMD_AEROBATIC_MANEUVER
    enum class Maneuver : uint8_t {
        NONE         = 0,
        AILERON_ROLL = 1,
        LOOP         = 2,
    };

    enum class State : uint8_t {
        IDLE = 0,
        ENTRY,
        ROLLING,
        EXIT,
        ABORT,
    };

    // why a start request was refused. The vehicle maps these onto
    // MAV_RESULT codes; the library does not include mavlink headers.
    enum class StartResult : uint8_t {
        OK = 0,
        ALREADY_RUNNING,
        BAD_MANEUVER,       // unknown or unimplemented maneuver ID
        ENVELOPE,           // entry envelope check failed
    };

    // why a running maneuver was given up on. Reported to the GCS and
    // otherwise unused: the recovery is the same whatever the cause.
    enum class AbortReason : uint8_t {
        NONE = 0,
        PILOT,          // pilot moved a stick
        ALTITUDE,       // fell below AEROB_ALT_MIN
        AIRSPEED,       // fell below AIRSPEED_MIN
        TIMEOUT,        // ROLLING ran long
        NOT_FLYING,
        MODE_CHANGE,    // left ACRO, via reset()
    };

    /*
      vehicle state the library cannot obtain for itself. Attitude and
      body rates come from AP::ahrs(); these do not, so the ACRO hook
      passes them in.

      Filled by Plane::get_aerobatics_state() for both callers, so the
      entry checks and the in-flight abort checks cannot drift apart.
     */
    struct VehicleState {
        float alt_agl;          // m above ground
        float airspeed;         // m/s, EAS
        float airspeed_min;     // m/s, AIRSPEED_MIN
        // true only when airspeed comes from a real sensor. A synthetic
        // estimate is rejected rather than trusted -- see check_envelope()
        bool airspeed_valid;
        bool is_flying;
        // pilot roll and pitch sticks, normalised to -1..1 with the
        // deadzone applied, so a centred stick reads exactly zero.
        // Non-zero enough and the pilot gets the aircraft straight back.
        float pilot_roll;
        float pilot_pitch;
    };

    // rate targets the ACRO hook writes over the pilot's, in deg/s
    struct Output {
        float roll_rate_dps;
        float pitch_rate_dps;
    };

    /*
      request a maneuver. Mode and arming are checked by the caller;
      everything else is checked here. On OK the state machine leaves
      IDLE and the ACRO hook takes over the rate targets.

      direction: negative for left, otherwise right
      reps:      number of full rotations, zero treated as one
      rate_dps:  commanded roll rate, zero to use AEROB_RATE
     */
    StartResult start(Maneuver m, float direction, float reps, float rate_dps,
                      const VehicleState &vs);

    /*
      advance the state machine by one loop. Returns true while a
      maneuver is running, in which case out holds the rate targets the
      ACRO hook must use in place of the pilot's; false means idle and
      the pilot keeps the sticks.

      vs is re-read every loop, not just at command time: altitude,
      airspeed and the sticks are abort triggers here.

      dt is the true loop timestep -- plane.G_Dt, never a hardcoded
      0.02: stabilize() is a FAST_TASK, so it runs at SCHED_LOOP_RATE
      and 50Hz is only the ArduPlane default.
     */
    bool update(float dt, const VehicleState &vs, Output &out);

    // return to IDLE, discarding any running maneuver. Called on ACRO
    // entry and exit so stale state never survives a mode change: this
    // object outlives the mode, so nothing else clears it.
    void reset(void);

    State get_state(void) const { return state; }

    // true while a maneuver is running and the ACRO hook should be
    // overriding the pilot rate targets
    bool active(void) const { return state != State::IDLE; }

    static const char *state_name(State s);
    const char *state_name(void) const { return state_name(state); }

private:

    // Parameters
    AP_Float rate;      // AEROB_RATE,    commanded body roll rate, deg/s
    AP_Float pitch;     // AEROB_PITCH,   entry pitch-up target, deg
    AP_Float alt_min;   // AEROB_ALT_MIN, altitude floor AGL, m

    // entry conditions, checked once at command time. Sends the reason
    // for a rejection to the GCS; the caller only sees pass or fail.
    bool check_envelope(const VehicleState &vs) const;

    // the abort triggers, tested every loop while a maneuver runs
    AbortReason check_aborts(const VehicleState &vs) const;

    void report_abort(AbortReason r) const;
    static const char *abort_reason_name(AbortReason r);

    /*
      fly both axes back to level, filling out with the rate demands and
      returning true once there. Shared by EXIT and ABORT: EXIT arrives
      near upright after a whole number of rolls, ABORT can arrive
      anywhere in one.
     */
    bool level_off(Output &out) const;

    void set_state(State s);

    State state = State::IDLE;

    // body-axis roll since ROLLING began, radians, signed with the
    // commanded direction. Zeroed there rather than at command time so
    // the ENTRY wings-levelling does not count toward the rotation.
    float roll_accumulated = 0;

    // when the current state began, for the per-state time bounds
    uint32_t state_ms = 0;

    // how long ROLLING may run before it is treated as a failure.
    // Sized in start() from the commanded rate, not from a constant.
    uint32_t rolling_ms_max = 0;

    // the running request
    struct {
        Maneuver maneuver;
        float direction;    // -1 or +1
        uint8_t reps;
        float rate_dps;     // resolved, never zero
    } current;
};

#endif // AP_AEROBATICS_ENABLED
