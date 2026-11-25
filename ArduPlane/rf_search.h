#pragma once

/*
  RF search (RFSEARCH flight mode): fly the mission like AUTO, and while the companion's RF reading
  (NAMED_VALUE_FLOAT "RF_dBm") is above a threshold, sweep the nose-mounted antenna every RFS_SPACING
  metres of track: a left leg to RFS_LEG_DEG left of the course, a right leg through the course to RFS_LEG_DEG
  right of it, then a turn back onto the leg. Both legs follow the same rules, at full bank:
  - a reading RFS_STOP_DB below RFS_THR_DBM beyond the strongest one is a hard stop for that direction;
  Once a sweep has started, sweeps carry on every RFS_SPACING while each one's strongest reading stays above
  RFS_THR_DBM less RFS_STOP_DB, whatever the nose reads on the leg in between.
  - at RFS_LEG_DEG a leg ends, unless the signal is still rising: it carries on while the leg's strongest
    reading is less than RFS_PAST_DEG behind the heading, so the peak ends up inside the swept arc.
  A left leg that went so far round that going on round the circle is shorter than the right leg and the turn
  back does that instead, with no right leg. The turn back after the right leg reverses the roll, unless that
  would be more than 180 deg: then it keeps rolling the same way round the full circle.
  With RFS_TYPE a circle instead, a sweep is one full-bank turn all the way round, left or right.

  RFSEARCH is not a separate mode object: it is mode_auto with its rf_search flag set, so every
  AUTO-specific check in ArduPlane (take-off, landing, altitude, failsafes by identity) still applies.
 */

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>

class RFSearch {
public:
    RFSearch();

    static const struct AP_Param::GroupInfo var_info[];

    // a reading from the companion's analyzer, dBm
    void handle_rf_dbm(float dbm);

    // on entering RFSEARCH: no sweep in progress, the first valid reading may start one
    void reset();

    // leaving RFSEARCH: end a sweep in progress (reported, so the companion closes it too)
    void abort(const char *why);

    // called from ModeAuto::navigate() in RFSEARCH at the navigation rate. leg_ok: on a plain
    // waypoint leg, not taking off, landing or in battery failsafe. Returns true while a sweep owns the
    // steering (the mission is then not updated)
    bool update(const Location &here, const Location &prev_wp, const Location &next_wp, bool leg_ok);

    bool sweeping() const { return _state != State::TRACK; }

private:
    enum class State : uint8_t {
        TRACK  = 0,     // following the mission leg
        LEFT   = 1,     // left leg: to RFS_LEG_DEG left of the course (or further, see above)
        RIGHT  = 2,     // right leg: through the course to RFS_LEG_DEG right of it
        // 3 was a fixed 90 deg turn back, no longer used
        RETURN = 4,     // back onto the leg
    };

    AP_Int8  _enable;
    AP_Float _thr_dbm;
    AP_Float _spacing_m;
    AP_Float _min_leg_m;
    AP_Float _stale_s;
    AP_Float _leg_deg;
    AP_Float _past_deg;
    AP_Float _stop_db;
    AP_Int8  _type;

    // latest reading and its smoothed level
    float    _smooth_dbm = -1000;
    uint32_t _last_rf_ms = 0;
    bool     _have_rf = false;

    // track distance since the last sweep
    Location _last_pos;
    bool     _have_last_pos;
    float    _dist_since_m;

    // sweep state
    State    _state;
    uint16_t _sweeps = 0;
    float    _course_deg = 0;       // leg course at the start of the sweep
    float    _phase_turn_deg = 0;   // turned so far in this phase, positive in the phase direction
    float    _phase_angle_deg = 0;  // how far the RETURN turn goes
    float    _off_deg = 0;          // heading relative to the course, positive right, not wrapped
    bool     _extending = false;    // this leg is past RFS_LEG_DEG, following a rising signal
    bool     _engaged = false;      // the last sweep peaked over RFS_THR_DBM less RFS_STOP_DB: sweep on regardless
    float    _sweep_peak_dbm = -1000;   // strongest smoothed reading of this sweep, all phases
    bool     _circle = false;       // RFS_TYPE circle: this sweep is one leg all the way round
    int8_t   _dir = 1;              // -1 left, +1 right
    float    _prev_yaw_deg = 0;
    uint32_t _last_report_ms;
    uint32_t _start_ms = 0;

    // readings during the legs, for the strongest one and the level at the current heading
    static const uint16_t MAX_SAMPLES = 200;
    struct Sample {
        float yaw_deg;
        float off_deg;          // _off_deg when taken
        float dbm;
    } _samples[MAX_SAMPLES];
    float    _smoothed[MAX_SAMPLES];   // scratch for find_peak()
    uint16_t _nsamples;
    uint16_t _leg_first;            // first sample of the current leg
    float    _max_dbm = -1000;
    float    _max_yaw_deg = 0;
    float    _leg_max_off_deg = 0;  // _off_deg of the current leg's strongest reading

    bool signal_valid() const;
    void start_sweep(const Location &here, const Location &next_wp);
    void start_leg(State state);
    void end_leg(const Location &here, const Location &next_wp, float yaw, const char *why);
    void end_sweep(const char *why);
    // smooths the readings over heading (against fading) and finds the strongest, of the sweep and of this leg
    void find_peak();
    // the rest of the way round, in the current direction, back onto the leg as it is now from here
    void circle_back(const Location &here, const Location &next_wp);
    void report_state();
    void log_state() const;
};
