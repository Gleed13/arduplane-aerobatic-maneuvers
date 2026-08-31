/*
  automatic aerobatic maneuvers for ArduPlane. See AP_Aerobatics.h.
 */

#include "AP_Aerobatics.h"

#if AP_AEROBATICS_ENABLED

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
