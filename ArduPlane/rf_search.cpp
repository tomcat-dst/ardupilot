#include "Plane.h"
#include "rf_search.h"

// longest a sweep may own the steering before it is abandoned
static const uint32_t SWEEP_TIMEOUT_MS = 90000;
// readings within this yaw of each other are averaged when looking for the peak (fading)
static const float PEAK_SMOOTH_DEG = 7.5f;
// a phase is complete when it has this little left to turn
static const float PHASE_DONE_DEG = 5.0f;
// EMA weight of a new reading in the smoothed level used for detection
static const float SMOOTH_ALPHA = 0.3f;

const AP_Param::GroupInfo RFSearch::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: RF search enable
    // @Description: Enables sweeps in RFSEARCH mode. With 0, RFSEARCH flies the mission exactly like AUTO.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, RFSearch, _enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: THR_DBM
    // @DisplayName: RF detection threshold
    // @Description: Smoothed RF_dBm reading above which a sweep is started.
    // @Units: dBm
    // @Range: -120 0
    // @User: Standard
    AP_GROUPINFO("THR_DBM", 2, RFSearch, _thr_dbm, -90.0f),

    // @Param: SPACING
    // @DisplayName: Sweep spacing
    // @Description: Track distance between sweeps while the signal is above the threshold.
    // @Units: m
    // @Range: 500 20000
    // @User: Standard
    AP_GROUPINFO("SPACING", 3, RFSearch, _spacing_m, 3000.0f),

    // @Param: MIN_LEG
    // @DisplayName: Minimum distance to the next waypoint
    // @Description: No sweep starts closer than this to the next waypoint, or closer than a third of it to the previous one.
    // @Units: m
    // @Range: 300 10000
    // @User: Standard
    AP_GROUPINFO("MIN_LEG", 4, RFSearch, _min_leg_m, 500.0f),

    // @Param: STALE
    // @DisplayName: RF reading timeout
    // @Description: A reading older than this does not count as a signal.
    // @Units: s
    // @Range: 0.5 10
    // @User: Advanced
    AP_GROUPINFO("STALE", 7, RFSearch, _stale_s, 2.0f),

    // 5, 6, 8 and 9 were PEAK_DB, PEAK_DEG, EXT_DB and EXT_DEG

    // @Param: LEG_DEG
    // @DisplayName: Sweep leg angle
    // @Description: How far either side of the course a sweep leg turns, unless the signal is still rising there (see RFS_PAST_DEG).
    // @Units: deg
    // @Range: 30 180
    // @User: Standard
    AP_GROUPINFO("LEG_DEG", 10, RFSearch, _leg_deg, 60.0f),

    // @Param: PAST_DEG
    // @DisplayName: Sweep past the peak
    // @Description: A sweep leg beyond RFS_LEG_DEG carries on while its strongest reading is less than this behind the heading, that is while the signal is still rising, so the peak ends up inside the swept arc. About half the antenna's beamwidth.
    // @Units: deg
    // @Range: 5 45
    // @User: Advanced
    AP_GROUPINFO("PAST_DEG", 11, RFSearch, _past_deg, 15.0f),

    // @Param: STOP_DB
    // @DisplayName: Sweep threshold hysteresis
    // @Description: Once a sweep has started, sweeps carry on every RFS_SPACING while each sweep's strongest smoothed reading stays above RFS_THR_DBM less this, whatever the reading on the leg; after a sweep peaks below that, the next one needs RFS_THR_DBM again. With legs, a reading this far below RFS_THR_DBM (beyond the strongest one) is also the hard stop for a leg. Keeps the search from stopping on the first fade.
    // @Units: dB
    // @Range: 0 20
    // @User: Advanced
    AP_GROUPINFO("STOP_DB", 12, RFSearch, _stop_db, 5.0f),

    // @Param: TYPE
    // @DisplayName: Sweep pattern
    // @Description: Left and right legs (see RFS_LEG_DEG), or one full circle at full bank, left or right, then back onto the leg.
    // @Values: 0:Legs,1:Circle left,2:Circle right
    // @User: Standard
    AP_GROUPINFO("TYPE", 13, RFSearch, _type, 2),

    AP_GROUPEND
};

RFSearch::RFSearch()
{
    AP_Param::setup_object_defaults(this, var_info);
    reset();
}

void RFSearch::reset()
{
    _state = State::TRACK;
    _have_last_pos = false;
    // the first valid reading starts a sweep straight away
    _dist_since_m = 1e7f;
    _engaged = false;
    _nsamples = 0;
    _leg_first = 0;
    _last_report_ms = 0;
}

bool RFSearch::signal_valid() const
{
    // a search under way (the last sweep's strongest reading was over the threshold less RFS_STOP_DB) carries
    // on whatever the nose antenna reads on the leg: the source is usually off to the side
    return _have_rf &&
           AP_HAL::millis() - _last_rf_ms < uint32_t(_stale_s * 1000) &&
           (_engaged || _smooth_dbm >= _thr_dbm);
}

void RFSearch::handle_rf_dbm(float dbm)
{
    const uint32_t now = AP_HAL::millis();
    if (!_have_rf || now - _last_rf_ms > uint32_t(_stale_s * 1000)) {
        _smooth_dbm = dbm;
    } else {
        _smooth_dbm += SMOOTH_ALPHA * (dbm - _smooth_dbm);
    }
    _have_rf = true;
    _last_rf_ms = now;

    if (_state != State::TRACK) {
        _sweep_peak_dbm = MAX(_sweep_peak_dbm, _smooth_dbm);
    }
    if (_state != State::TRACK && _state != State::RETURN && _nsamples < MAX_SAMPLES) {
        _samples[_nsamples++] = { wrap_360(AP::ahrs().get_yaw_deg()), _off_deg, dbm };
    }
}

/*
  readings averaged over neighbours within PEAK_SMOOTH_DEG of heading (against fading) into _smoothed[],
  and the strongest of them
 */
void RFSearch::find_peak()
{
    float max_dbm = -1000;
    float max_yaw = 0;
    float leg_max_dbm = -1000;
    for (uint16_t i = 0; i < _nsamples; i++) {
        float sum = 0;
        uint16_t n = 0;
        for (uint16_t j = 0; j < _nsamples; j++) {
            if (fabsf(wrap_180(_samples[j].yaw_deg - _samples[i].yaw_deg)) <= PEAK_SMOOTH_DEG) {
                sum += _samples[j].dbm;
                n++;
            }
        }
        _smoothed[i] = sum / n;
        if (_smoothed[i] > max_dbm) {
            max_dbm = _smoothed[i];
            max_yaw = _samples[i].yaw_deg;
        }
        if (i >= _leg_first && _smoothed[i] > leg_max_dbm) {
            leg_max_dbm = _smoothed[i];
            _leg_max_off_deg = _samples[i].off_deg;
        }
    }
    _max_dbm = max_dbm;
    _max_yaw_deg = max_yaw;
}

void RFSearch::start_leg(State state)
{
    _state = state;
    _dir = state == State::LEFT ? -1 : 1;
    _extending = false;
    _leg_first = _nsamples;
    _leg_max_off_deg = _off_deg;
    report_state();
    log_state();
}

void RFSearch::start_sweep(const Location &here, const Location &next_wp)
{
    _sweeps++;
    _sweep_peak_dbm = _smooth_dbm;
    _course_deg = here.get_bearing_to(next_wp) * 0.01f;
    _prev_yaw_deg = wrap_360(AP::ahrs().get_yaw_deg());
    _off_deg = wrap_180(_prev_yaw_deg - _course_deg);
    _nsamples = 0;
    _max_dbm = -1000;
    _max_yaw_deg = 0;
    _start_ms = AP_HAL::millis();
    _dist_since_m = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u, course %.0f, %.1f dBm",
                    unsigned(_sweeps), double(_course_deg), double(_smooth_dbm));
    // a circle is one leg all the way round
    _circle = _type != 0;
    start_leg(_type == 2 ? State::RIGHT : State::LEFT);
}

/*
  a leg is over: the left one hands over to the right one, the right one to the turn back onto the leg
 */
void RFSearch::end_leg(const Location &here, const Location &next_wp, float yaw, const char *why)
{
    if (why != nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u %s at %.0f", unsigned(_sweeps), why, double(yaw));
    }
    if (_state == State::LEFT) {
        // the right leg sweeps back across to RFS_LEG_DEG right and turns back again; when going on round
        // is shorter than that, go on round (it sees the right side too)
        const float reach = -_off_deg;
        if (360 - reach <= reach + 2 * _leg_deg) {
            gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u full circle from %.0f left", unsigned(_sweeps), double(reach));
            circle_back(here, next_wp);
        } else {
            start_leg(State::RIGHT);
        }
        return;
    }
    // back onto the leg as it is now from here, not the course the sweep started on
    const float course = here.get_bearing_to(next_wp) * 0.01f;
    _off_deg -= wrap_180(course - _course_deg);
    _course_deg = course;
    // how far past the course we are in the direction we are rolling: turn back against the roll, unless
    // that is more than 180 deg (then keep rolling round the full circle) or the course is still ahead
    const float past = _dir * _off_deg;
    if (past <= 0) {
        _phase_angle_deg = -past;
    } else if (past > 180) {
        _phase_angle_deg = 360 - past;
        gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u full circle back to course", unsigned(_sweeps));
    } else {
        _phase_angle_deg = past;
        _dir = -_dir;
    }
    _phase_turn_deg = 0;
    _state = State::RETURN;
    report_state();
    log_state();
}

void RFSearch::circle_back(const Location &here, const Location &next_wp)
{
    const float course = here.get_bearing_to(next_wp) * 0.01f;
    _off_deg -= wrap_180(course - _course_deg);
    _course_deg = course;
    _phase_angle_deg = MAX(360 - _dir * _off_deg, 0);
    _phase_turn_deg = 0;
    _state = State::RETURN;
    report_state();
    log_state();
}

void RFSearch::abort(const char *why)
{
    if (_state != State::TRACK) {
        end_sweep(why);
    }
}

void RFSearch::end_sweep(const char *why)
{
    gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u %s, %u readings, peak %.1f dBm",
                    unsigned(_sweeps), why, unsigned(_nsamples), double(_sweep_peak_dbm));
    // hysteresis on what the whole sweep saw, not on the heading the leg happens to point at
    const bool engaged = _sweep_peak_dbm >= _thr_dbm - _stop_db;
    if (_engaged && !engaged) {
        gcs().send_text(MAV_SEVERITY_INFO, "RFS: peak under %.0f dBm, search paused", double(_thr_dbm - _stop_db));
    }
    _engaged = engaged;
    _state = State::TRACK;
    _dist_since_m = 0;
    report_state();
    log_state();
}

bool RFSearch::update(const Location &here, const Location &prev_wp, const Location &next_wp, bool leg_ok)
{
    const uint32_t now = AP_HAL::millis();

    if (_have_last_pos) {
        _dist_since_m += _last_pos.get_distance(here);
    }
    _last_pos = here;
    _have_last_pos = true;

    if (now - _last_report_ms >= 1000) {
        report_state();
    }

    if (_state == State::TRACK) {
        if (_enable && leg_ok && signal_valid() &&
            _dist_since_m >= _spacing_m &&
            here.get_distance(next_wp) >= _min_leg_m &&
            here.get_distance(prev_wp) >= _min_leg_m / 3) {
            start_sweep(here, next_wp);
        } else {
            return false;
        }
    }

    if (!_enable || !leg_ok) {
        end_sweep("aborted");
        return false;
    }
    if (now - _start_ms > SWEEP_TIMEOUT_MS) {
        end_sweep("timed out");
        return false;
    }

    // turned since the last update
    const float yaw = wrap_360(AP::ahrs().get_yaw_deg());
    const float turned = wrap_180(yaw - _prev_yaw_deg);
    _prev_yaw_deg = yaw;
    _off_deg += turned;

    if (_state == State::LEFT || _state == State::RIGHT) {
        // the same rules for both legs; reach: how far this leg's side of the course we are
        const float reach = _dir * _off_deg;
        bool below = false, rising = false;
        if (_nsamples - _leg_first >= 3) {
            find_peak();
            const float level = _smoothed[_nsamples - 1];
            // below the threshold (less the margin) beyond the strongest reading in the leg's direction: a hard stop
            below = level < _thr_dbm - _stop_db && _dir * wrap_180(yaw - _max_yaw_deg) > PEAK_SMOOTH_DEG;
            // the leg's strongest reading is recent: the signal is still rising, the peak may be further round
            rising = _max_dbm >= _thr_dbm && _dir * (_off_deg - _leg_max_off_deg) < _past_deg;
        }
        if (_circle) {
            // the last quarter rolls out onto the leg as it is now from here
            if (reach >= 270) {
                circle_back(here, next_wp);
            }
        } else if (reach >= 360 - PHASE_DONE_DEG) {
            // followed a rising signal all the way round: that is everything there is to see
            gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u rising all round", unsigned(_sweeps));
            circle_back(here, next_wp);
        } else if (below) {
            end_leg(here, next_wp, yaw, "below threshold");
        } else if (reach >= _leg_deg - PHASE_DONE_DEG) {
            if (!rising) {
                end_leg(here, next_wp, yaw, _extending ? "past the peak" : nullptr);
            } else if (!_extending) {
                _extending = true;
                gcs().send_text(MAV_SEVERITY_INFO, "RFS: sweep %u rising at %.0f, extending",
                                unsigned(_sweeps), double(_leg_deg.get()));
            }
        }
    } else if (_state == State::RETURN) {
        _phase_turn_deg += _dir * turned;
    }

    if (_state == State::RETURN && _phase_angle_deg - _phase_turn_deg < PHASE_DONE_DEG) {
        end_sweep("done");
        return false;
    }

    // a heading demand at most 90 deg ahead keeps the turn direction unambiguous. The legs hold full
    // bank to their end (a reversal or the turn back); only the turn back rolls out onto the course
    const float step = _state == State::RETURN ? constrain_float(_phase_angle_deg - _phase_turn_deg, 0, 90) : 90;
    plane.nav_controller->update_heading_hold(int32_t(wrap_360(yaw + _dir * step) * 100));
    return true;
}

void RFSearch::report_state()
{
    _last_report_ms = AP_HAL::millis();
    // the companion splits its readings into sweeps on this: 0 tracking, 1..4 sweep phases
    gcs().send_named_float("RFS_SWP", float(uint8_t(_state)));
}

void RFSearch::log_state() const
{
    // @LoggerMessage: RFS
    // @Description: RF search sweep state
    // @Field: TimeUS: Time since system startup
    // @Field: St: state, 0 tracking, 1 left leg, 2 right leg, 4 turn back onto the leg
    // @Field: N: sweep number
    // @Field: Crs: course the sweep is measured from (bearing to the next waypoint)
    // @Field: Yaw: heading
    // @Field: dBm: smoothed RF reading
    // @Field: Max: strongest smoothed reading of the sweep
    // @Field: MaxY: heading of the strongest reading
    AP::logger().Write("RFS", "TimeUS,St,N,Crs,Yaw,dBm,Max,MaxY", "QBHfffff",
                       AP_HAL::micros64(),
                       uint8_t(_state),
                       _sweeps,
                       _course_deg,
                       wrap_360(AP::ahrs().get_yaw_deg()),
                       _smooth_dbm,
                       _max_dbm,
                       _max_yaw_deg);
}
