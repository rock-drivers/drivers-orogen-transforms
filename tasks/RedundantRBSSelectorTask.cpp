/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RedundantRBSSelectorTask.hpp"

using namespace transforms;
using namespace base;

bool rbsIsValid(samples::RigidBodyState const& rbs);

RedundantRBSSelectorTask::RedundantRBSSelectorTask(std::string const& name)
    : RedundantRBSSelectorTaskBase(name)
{
    _init_timeout.set(base::Time::fromSeconds(1));
    _main_source_hysteresis.set(base::Time::fromSeconds(30));
    _main_gps_rtk_fixed_timeout.set(base::Time::fromSeconds(5));
}

RedundantRBSSelectorTask::~RedundantRBSSelectorTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RedundantRBSSelectorTask.hpp for more detailed
// documentation about them.

bool RedundantRBSSelectorTask::configureHook()
{
    if (!RedundantRBSSelectorTaskBase::configureHook())
        return false;
    m_source_timeout = _source_timeout.get();
    m_main_source_hysteresis = _main_source_hysteresis.get();
    m_position_error_threshold = _position_error_threshold.get();
    m_angles_error_thresholds = _angles_error_thresholds.get();
    m_main_gps_rtk_fixed_timeout = _main_gps_rtk_fixed_timeout.get();
    m_using_gnss_heading = _using_gnss_heading.get();

    return true;
}
bool RedundantRBSSelectorTask::startHook()
{
    if (!RedundantRBSSelectorTaskBase::startHook())
        return false;

    m_internal_state = InternalState();
    m_internal_state.init_deadline = base::Time::now() + _init_timeout.get();
    return true;
}

bool isMainGNSSHeadingValid(RedundantRBSSelectorTask::InternalState const& state)
{
    return base::Time::now() < state.main_rtk_fixed_deadline;
}

bool isMainSourceValid(RedundantRBSSelectorTask::InternalState const& state)
{
    return base::Time::now() < state.main_source_deadline;
}

bool isSecondarySourceValid(RedundantRBSSelectorTask::InternalState const& state)
{
    return base::Time::now() < state.secondary_source_deadline;
}

RedundantRBSSelectorTask::States updateState(
    RedundantRBSSelectorTask::States current_state,
    RedundantRBSSelectorTask::InternalState const& internal,
    bool using_gnss_heading)
{
    // for readability's sake
    using States = RedundantRBSSelectorTask::States;
    base::Time now = base::Time::now();

    bool main_valid = isMainSourceValid(internal);
    bool secondary_valid = isSecondarySourceValid(internal);
    bool gnss_heading_valid = isMainGNSSHeadingValid(internal);
    bool init = RedundantRBSSelectorTask::initializing(current_state, internal);

    // gnss heading is not checked here since we still want to forward main without
    // without gnss heading when secondary is not available
    if (!init && !main_valid && !secondary_valid) {
        return States::NO_VALID_SOURCES;
    }

    switch (current_state) {
        case States::RUNNING:
            if (init) {
                if (main_valid && secondary_valid) {
                    return States::BOTH_SOURCES_VALID;
                }
                return States::RUNNING;
            }
            // !init -> fall through to BOTH_SOURCES_VALID
        case States::BOTH_SOURCES_VALID:
            if (!main_valid) {
                return States::INVALID_MAIN_SOURCE;
            }
            else if (!secondary_valid) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            else if (using_gnss_heading && !gnss_heading_valid) {
                return States::INVALID_MAIN_SOURCE_GNSS_HEADING;
            }
            return States::BOTH_SOURCES_VALID;
        case States::MAIN_SOURCE_RECOVERING:
            if (!main_valid) {
                return States::INVALID_MAIN_SOURCE;
            }
            else if (!secondary_valid) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            else if (using_gnss_heading && !gnss_heading_valid) {
                return States::INVALID_MAIN_SOURCE_GNSS_HEADING;
            }
            else if (now > internal.hysteresis_deadline) {
                return States::BOTH_SOURCES_VALID;
            }

            return States::MAIN_SOURCE_RECOVERING;
        case States::INVALID_MAIN_SOURCE_GNSS_HEADING:
            if (!main_valid) {
                return States::INVALID_MAIN_SOURCE;
            }
            else if (!secondary_valid) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            if (gnss_heading_valid) {
                // goes to main source recovering and enter the hysteresis logic
                return States::MAIN_SOURCE_RECOVERING;
            }
            return States::INVALID_MAIN_SOURCE_GNSS_HEADING;
        case States::INVALID_MAIN_SOURCE:
            if (main_valid) {
                return States::MAIN_SOURCE_RECOVERING;
            }
            return States::INVALID_MAIN_SOURCE;
        case States::INVALID_SECONDARY_SOURCE:
            if (secondary_valid) {
                return States::BOTH_SOURCES_VALID;
            }
            return States::INVALID_SECONDARY_SOURCE;
        case States::NO_VALID_SOURCES:
            return States::NO_VALID_SOURCES;
        default:
            throw std::invalid_argument("invalid controller state");
    }
}
bool RedundantRBSSelectorTask::initializing(States state, InternalState const& internal)
{
    return (state == States::RUNNING) && (base::Time::now() < internal.init_deadline);
}

bool RedundantRBSSelectorTask::initializing() const
{
    return initializing(state(), m_internal_state);
}

bool RedundantRBSSelectorTask::updateMainSource(samples::RigidBodyState const& rbs)
{
    if (rbsIsValid(rbs)) {
        m_internal_state.main_source_deadline = base::Time::now() + m_source_timeout;
        return true;
    }
    return false;
}

bool RedundantRBSSelectorTask::updateSecondarySource(samples::RigidBodyState const& rbs)
{
    if (rbsIsValid(rbs)) {
        m_internal_state.secondary_source_deadline = base::Time::now() + m_source_timeout;
        return true;
    }
    return false;
}

bool RedundantRBSSelectorTask::updateMainGPS(gps_base::Solution const& solution)
{
    if (solution.positionType == gps_base::GPS_SOLUTION_TYPES::RTK_FIXED) {
        m_internal_state.main_rtk_fixed_deadline =
            base::Time::now() + m_main_gps_rtk_fixed_timeout;
        return true;
    }

    m_internal_state.main_rtk_fixed_deadline = base::Time();
    return false;
}

void RedundantRBSSelectorTask::updateHook()
{
    RedundantRBSSelectorTaskBase::updateHook();

    gps_base::Solution main_gps;
    if (_main_gps.read(main_gps, false) == RTT::NewData) {
        updateMainGPS(main_gps);
    }

    bool main_is_new_and_valid{false};
    samples::RigidBodyState main_rbs;
    if (_main_rbs_source.read(main_rbs) == RTT::NewData) {
        main_is_new_and_valid = updateMainSource(main_rbs);
    }

    bool secondary_is_new_and_valid{false};
    samples::RigidBodyState secondary_rbs;
    if (_secondary_rbs_source.read(secondary_rbs) == RTT::NewData) {
        secondary_is_new_and_valid = updateSecondarySource(secondary_rbs);
    }

    if (main_is_new_and_valid || secondary_is_new_and_valid) {
        if (rbsIsValid(main_rbs) && rbsIsValid(secondary_rbs)) {
            PoseDivergence pose_divergence = checkDivergences(main_rbs, secondary_rbs);
            pose_divergence.main_gps = GPSInfo::fromSolution(main_gps);
            _pose_divergence.write(pose_divergence);
        }
    }

    switch (state()) {
        case States::MAIN_SOURCE_RECOVERING:
            if (m_internal_state.hysteresis_deadline.isNull()) {
                m_internal_state.hysteresis_deadline =
                    base::Time::now() + m_main_source_hysteresis;
            }
            break;
        case States::RUNNING:
        case States::BOTH_SOURCES_VALID:
        case States::INVALID_MAIN_SOURCE:
        case States::INVALID_SECONDARY_SOURCE:
        case States::INVALID_MAIN_SOURCE_GNSS_HEADING:
        case States::NO_VALID_SOURCES:
            m_internal_state.hysteresis_deadline = base::Time();
            break;
        default:
            throw std::invalid_argument("invalid controller state");
    }

    States next_state = updateState(state(), m_internal_state, m_using_gnss_heading);
    if (next_state == States::NO_VALID_SOURCES) {
        return exception(States::NO_VALID_SOURCES);
    }

    if (next_state != state() && next_state != States::NO_VALID_SOURCES) {
        state(next_state);
    }

    if (next_state == States::BOTH_SOURCES_VALID ||
        next_state == States::INVALID_SECONDARY_SOURCE) {
        if (main_is_new_and_valid) {
            _rbs_out.write(main_rbs);
        }
    }
    else if (!initializing() && secondary_is_new_and_valid) {
        _rbs_out.write(secondary_rbs);
    }
}

PoseDivergence RedundantRBSSelectorTask::checkDivergences(
    samples::RigidBodyState const& first,
    samples::RigidBodyState const& second)
{
    PoseDivergence divergence;
    divergence.time = Time::now();
    divergence.position_error = (first.position - second.position).norm();
    divergence.position_divergent =
        divergence.position_error > m_position_error_threshold;

    Vector3d diff_orientation =
        getEuler(first.orientation) - getEuler(second.orientation);

    divergence.roll_error = Angle::fromRad(std::fabs(diff_orientation[2]));
    divergence.pitch_error = Angle::fromRad(std::fabs(diff_orientation[1]));
    divergence.yaw_error = Angle::fromRad(std::fabs(diff_orientation[0]));
    divergence.roll_divergent = divergence.roll_error > m_angles_error_thresholds.roll;
    divergence.pitch_divergent = divergence.pitch_error > m_angles_error_thresholds.pitch;
    divergence.yaw_divergent = divergence.yaw_error > m_angles_error_thresholds.yaw;
    return divergence;
}

void RedundantRBSSelectorTask::errorHook()
{
    RedundantRBSSelectorTaskBase::errorHook();
}
void RedundantRBSSelectorTask::stopHook()
{
    RedundantRBSSelectorTaskBase::stopHook();
}
void RedundantRBSSelectorTask::cleanupHook()
{
    RedundantRBSSelectorTaskBase::cleanupHook();
}

bool rbsIsValid(samples::RigidBodyState const& rbs)
{
    if (rbs.hasValidPosition() && rbs.hasValidVelocity() &&
        rbs.hasValidAngularVelocity() && rbs.hasValidOrientation()) {
        return true;
    }

    return false;
}
