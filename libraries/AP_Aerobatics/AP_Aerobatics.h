#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_AEROBATICS_ENABLED
#define AP_AEROBATICS_ENABLED 1
#endif

#if AP_AEROBATICS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

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

    State state = State::IDLE;
};

#endif // AP_AEROBATICS_ENABLED
