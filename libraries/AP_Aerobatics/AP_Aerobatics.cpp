/*
  automatic aerobatic maneuvers for ArduPlane. See AP_Aerobatics.h.
 */

#include "AP_Aerobatics.h"

#if AP_AEROBATICS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// clamps on the commanded roll rate, whether it comes from param4 or
// from AEROB_RATE
#define AEROBATICS_RATE_MIN          30.0   // deg/s
#define AEROBATICS_RATE_MAX         360.0   // deg/s

// most repetitions we will accept in one command
#define AEROBATICS_REPS_MAX             5

// defaults chosen for the Extra 300L in RealFlight; all three exist so
// they can be tuned in flight without a rebuild
#define AEROBATICS_RATE_DEFAULT     180.0   // deg/s
#define AEROBATICS_PITCH_DEFAULT     25.0   // deg
#define AEROBATICS_ALT_MIN_DEFAULT   50.0   // m AGL

const AP_Param::GroupInfo AP_Aerobatics::var_info[] = {

    // @Param: RATE
    // @DisplayName: Aerobatics roll rate
    // @Description: Commanded body-axis roll rate during an automatic aileron roll. Used when param4 of MAV_CMD_AEROBATIC_MANEUVER is zero.
    // @Range: 30 360
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("RATE", 1, AP_Aerobatics, rate, AEROBATICS_RATE_DEFAULT),

    // @Param: PITCH
    // @DisplayName: Aerobatics entry pitch
    // @Description: Pitch-up target held during the ENTRY phase, before the roll starts.
    // @Range: 0 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PITCH", 2, AP_Aerobatics, pitch, AEROBATICS_PITCH_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Aerobatics minimum altitude
    // @Description: Altitude floor above ground. Checked once as an entry condition, then continuously during the maneuver as an abort trigger.
    // @Range: 20 200
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 3, AP_Aerobatics, alt_min, AEROBATICS_ALT_MIN_DEFAULT),

    AP_GROUPEND
};

AP_Aerobatics::StartResult AP_Aerobatics::start(Maneuver m, float direction, float reps,
                                                float rate_dps, const VehicleState &vs)
{
    if (active()) {
        return StartResult::ALREADY_RUNNING;
    }

    // only the aileron roll is implemented; the loop is a stretch goal
    if (m != Maneuver::AILERON_ROLL) {
        return StartResult::BAD_MANEUVER;
    }

    if (!check_envelope(vs)) {
        return StartResult::ENVELOPE;
    }

    current.maneuver = m;
    current.direction = is_negative(direction) ? -1.0 : 1.0;

    // zero repetitions is taken as one, so "long 31010 1 1 0 0 ..." does
    // something sensible rather than nothing
    const float r = is_positive(reps) ? reps : 1.0;
    current.reps = constrain_int16(int16_t(r), 1, AEROBATICS_REPS_MAX);

    // param4 of zero means "use AEROB_RATE"
    const float requested = is_positive(rate_dps) ? rate_dps : rate.get();
    current.rate_dps = constrain_float(requested, AEROBATICS_RATE_MIN, AEROBATICS_RATE_MAX);

    set_state(State::ENTRY);

    gcs().send_text(MAV_SEVERITY_INFO, "AERO: roll %s x%u at %.0f deg/s",
                    is_negative(current.direction) ? "left" : "right",
                    unsigned(current.reps), double(current.rate_dps));

    return StartResult::OK;
}

void AP_Aerobatics::reset(void)
{
    if (state != State::IDLE) {
        set_state(State::IDLE);
    }
}

/*
  entry envelope. Checked once, at command time.

  TODO(stage 3): airspeed above 1.3 * ARSPD_FBW_MIN, altitude above
  AEROB_ALT_MIN, roll and pitch within ~20 deg of level, armed and
  flying. Accepts everything for now so the command plumbing can be
  tested on its own.
 */
bool AP_Aerobatics::check_envelope(const VehicleState &vs) const
{
    (void)vs;
    return true;
}

void AP_Aerobatics::set_state(State s)
{
    if (s == state) {
        return;
    }
    state = s;
    gcs().send_text(MAV_SEVERITY_INFO, "AERO: %s", state_name(s));
}

const char *AP_Aerobatics::state_name(State s)
{
    switch (s) {
    case State::IDLE:
        return "IDLE";
    case State::ENTRY:
        return "ENTRY";
    case State::ROLLING:
        return "ROLLING";
    case State::EXIT:
        return "EXIT";
    case State::ABORT:
        return "ABORT";
    }
    return "UNKNOWN";
}

#endif // AP_AEROBATICS_ENABLED
