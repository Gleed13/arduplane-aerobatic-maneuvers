/*
  automatic aerobatic maneuvers for ArduPlane. See AP_Aerobatics.h.
 */

#include "AP_Aerobatics.h"

#if AP_AEROBATICS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

// clamps on the commanded roll rate, whether it comes from param4 or
// from AEROB_RATE
#define AEROBATICS_RATE_MIN          30.0   // deg/s
#define AEROBATICS_RATE_MAX         360.0   // deg/s

// most repetitions we will accept in one command
#define AEROBATICS_REPS_MAX             5

/*
  ENTRY and EXIT both fly an attitude to a target, which the rate
  controllers below need as a rate demand. One proportional gain, in
  1/s, converts the remaining attitude error in degrees into deg/s.

  This duplicates ArduPlane's own attitude loop, which uses
  angle_err_deg / gains.tau (RLL2SRV_TCONST, default 0.5s, so 2.0).
  AUTOTUNE adjusts tau and will not adjust this. See PLAN.md stage 7.
 */
#define AEROBATICS_LEVEL_P             2.0
#define AEROBATICS_PITCH_RATE_MAX     45.0   // deg/s
#define AEROBATICS_LEVEL_ROLL_RATE_MAX 90.0  // deg/s

// ENTRY is done at this much of AEROB_PITCH, EXIT when both axes are
// this close to level and the roll has actually stopped
#define AEROBATICS_ENTRY_PITCH_TOL     3.0   // deg
#define AEROBATICS_EXIT_TOL            5.0   // deg
#define AEROBATICS_EXIT_RATE_TOL      20.0   // deg/s

/*
  bounds on the two attitude-seeking states, so neither can hang if the
  target is never reached -- a nose-up target the aircraft cannot hold,
  or a level-off fighting a trim offset. Reaching one is not an abort,
  it just moves the machine on. Stage 6 adds the real abort triggers.
 */
#define AEROBATICS_ENTRY_MS_MAX       3000
#define AEROBATICS_EXIT_MS_MAX        2000

// entry envelope. Margin over AIRSPEED_MIN, and how far from level the
// aircraft may be when the command arrives.
#define AEROBATICS_ENTRY_ASPD_RATIO   1.3
#define AEROBATICS_ENTRY_LEVEL_DEG   20.0

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

    roll_accumulated = 0;
    set_state(State::ENTRY);

    gcs().send_text(MAV_SEVERITY_INFO, "AERO: roll %s x%u at %.0f deg/s",
                    is_negative(current.direction) ? "left" : "right",
                    unsigned(current.reps), double(current.rate_dps));

    return StartResult::OK;
}

/*
  one loop of the state machine. The ACRO hook calls this in place of
  computing the pilot rate targets, and uses out only when this returns
  true.
 */
bool AP_Aerobatics::update(float dt, Output &out)
{
    if (state == State::IDLE) {
        return false;
    }

    const AP_AHRS &ahrs = AP::ahrs();

    // zero unless a state below asks for something, so a state that
    // commands nothing hands back level rather than a stale demand
    out.roll_rate_dps = 0;
    out.pitch_rate_dps = 0;

    const uint32_t state_elapsed = AP_HAL::millis() - state_ms;

    switch (state) {

    case State::IDLE:
        // unreachable, handled above
        break;

    case State::ENTRY: {
        /*
          raise the nose to AEROB_PITCH before rolling, so the aircraft
          spends the roll descending back through level rather than
          starting there and finishing well below it.
         */
        const float pitch_err = radians(pitch.get()) - ahrs.get_pitch_rad();
        out.pitch_rate_dps = constrain_float(degrees(pitch_err) * AEROBATICS_LEVEL_P,
                                             -AEROBATICS_PITCH_RATE_MAX,
                                             AEROBATICS_PITCH_RATE_MAX);
        // hold the wings level on the way up
        out.roll_rate_dps = constrain_float(-degrees(ahrs.get_roll_rad()) * AEROBATICS_LEVEL_P,
                                            -AEROBATICS_LEVEL_ROLL_RATE_MAX,
                                            AEROBATICS_LEVEL_ROLL_RATE_MAX);

        if (fabsf(pitch_err) <= radians(AEROBATICS_ENTRY_PITCH_TOL) ||
            state_elapsed >= AEROBATICS_ENTRY_MS_MAX) {
            /*
              the count starts here, not at command time: levelling the
              wings during ENTRY turns the aircraft about the roll axis
              too, and that rotation is not part of the maneuver.
             */
            roll_accumulated = 0;
            set_state(State::ROLLING);
        }
        break;
    }

    case State::ROLLING:
        /*
          integrate the body-axis roll rate. Do NOT accumulate deltas of
          ahrs.roll: that wraps at +-180 so passing through inverted
          jumps the count backwards by nearly a turn and the maneuver
          never ends, and Euler roll rate carries a tan(pitch) term that
          is already wrong at the entry pitch and singular at 90 degrees.
         */
        roll_accumulated += ahrs.get_gyro().x * dt;

        // a pure aileron roll holds neutral elevator, so pitch rate
        // stays at the zero set above
        out.roll_rate_dps = current.direction * current.rate_dps;

        if (fabsf(roll_accumulated) >= current.reps * M_2PI) {
            out.roll_rate_dps = 0;
            set_state(State::EXIT);
        }
        break;

    case State::EXIT: {
        /*
          fly back to level on both axes. get_roll_rad() is already
          wrapped to +-180, so negating it is the shortest way round.
         */
        const float roll_err = -ahrs.get_roll_rad();
        const float pitch_err = -ahrs.get_pitch_rad();
        out.roll_rate_dps = constrain_float(degrees(roll_err) * AEROBATICS_LEVEL_P,
                                            -AEROBATICS_LEVEL_ROLL_RATE_MAX,
                                            AEROBATICS_LEVEL_ROLL_RATE_MAX);
        out.pitch_rate_dps = constrain_float(degrees(pitch_err) * AEROBATICS_LEVEL_P,
                                             -AEROBATICS_PITCH_RATE_MAX,
                                             AEROBATICS_PITCH_RATE_MAX);

        /*
          the rate condition matters as much as the angles: roll passes
          through level at full rate on the way out, and handing back
          there leaves acro_locking to hold whatever bank it inherits.
         */
        const bool level = fabsf(roll_err) <= radians(AEROBATICS_EXIT_TOL) &&
                           fabsf(pitch_err) <= radians(AEROBATICS_EXIT_TOL) &&
                           fabsf(ahrs.get_gyro().x) <= radians(AEROBATICS_EXIT_RATE_TOL);

        if (level || state_elapsed >= AEROBATICS_EXIT_MS_MAX) {
            gcs().send_text(MAV_SEVERITY_INFO, "AERO: done, roll=%.0f roll_err=%.0f pitch=%.0f",
                            double(degrees(roll_accumulated)),
                            double(degrees(roll_err)),
                            double(degrees(ahrs.get_pitch_rad())));
            set_state(State::IDLE);
            return false;
        }
        break;
    }

    case State::ABORT:
        // stage 6 fills this in; nothing reaches it yet
        set_state(State::IDLE);
        return false;
    }

    return true;
}

void AP_Aerobatics::reset(void)
{
    roll_accumulated = 0;
    if (state != State::IDLE) {
        set_state(State::IDLE);
    }
}

/*
  entry envelope. Checked once, at command time; arming and mode are the
  caller's business. Each failure sends its own text, so a rejection in
  MAVProxy says which condition was missed rather than just "envelope".

  The altitude and airspeed conditions are checked again continuously
  while the maneuver runs, where they abort rather than reject.
 */
bool AP_Aerobatics::check_envelope(const VehicleState &vs) const
{
    const AP_AHRS &ahrs = AP::ahrs();

    if (!vs.is_flying) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AERO: not flying");
        return false;
    }

    if (vs.alt_agl < alt_min) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AERO: alt %.0fm below %.0fm",
                        double(vs.alt_agl), double(alt_min.get()));
        return false;
    }

    /*
      a synthetic airspeed estimate is a throttle-and-attitude guess, so
      the gate would pass or fail for reasons that have nothing to do
      with airspeed. Refuse rather than trust it -- this needs
      ARSPD_USE 1, which the FlightAxis backend does not set.
     */
    if (!vs.airspeed_valid) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AERO: no airspeed sensor, set ARSPD_USE 1");
        return false;
    }

    const float airspeed_entry = AEROBATICS_ENTRY_ASPD_RATIO * vs.airspeed_min;
    if (vs.airspeed < airspeed_entry) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AERO: airspeed %.0f below %.0f",
                        double(vs.airspeed), double(airspeed_entry));
        return false;
    }

    // start from a known attitude, so the roll is about the flight path
    // rather than whatever bank the aircraft happened to be holding
    const float level_max = radians(AEROBATICS_ENTRY_LEVEL_DEG);
    if (fabsf(ahrs.get_roll_rad()) > level_max || fabsf(ahrs.get_pitch_rad()) > level_max) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AERO: not level, roll=%.0f pitch=%.0f",
                        double(degrees(ahrs.get_roll_rad())),
                        double(degrees(ahrs.get_pitch_rad())));
        return false;
    }

    return true;
}

void AP_Aerobatics::set_state(State s)
{
    if (s == state) {
        return;
    }
    state = s;
    // ENTRY and EXIT are both bounded, so every state needs its start time
    state_ms = AP_HAL::millis();
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
