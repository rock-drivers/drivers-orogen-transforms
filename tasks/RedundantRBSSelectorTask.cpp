/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RedundantRBSSelectorTask.hpp"

using namespace transforms;
using namespace base;

bool rbsIsValid(samples::RigidBodyState const& rbs);

RedundantRBSSelectorTask::RedundantRBSSelectorTask(std::string const& name)
    : RedundantRBSSelectorTaskBase(name)
{
    _init_timeout.set(base::Time::fromSeconds(1));
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
    m_main_source_hysteresis = _source_timeout.get();
    m_position_threshold = _position_threshold.get();
    m_angle_error_thresholds = _angle_error_thresholds.get();
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

bool isMainSourceValid(RedundantRBSSelectorTask::InternalState const& state)
{
    return base::Time::now() < state.main_source_deadline;
}

bool isSecondarySourceValid(RedundantRBSSelectorTask::InternalState const& state)
{
    return base::Time::now() < state.secondary_source_deadline;
}

bool isDifferenceOutsideThreshold(double first, double second, double threshold)
{
    return std::abs(first - second) > threshold;
}

RedundantRBSSelectorTask::States updateState(
    RedundantRBSSelectorTask::States current_state,
    RedundantRBSSelectorTask::InternalState const& internal)
{
    // for readability's sake
    using States = RedundantRBSSelectorTask::States;
    base::Time now = base::Time::now();

    bool main_valid = isMainSourceValid(internal);
    bool secondary_valid = isSecondarySourceValid(internal);
    bool init = (current_state == States::RUNNING) &&
                (base::Time::now() < internal.init_deadline);

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
            return States::BOTH_SOURCES_VALID;
        case States::MAIN_SOURCE_RECOVERING:
            if (!main_valid) {
                return States::INVALID_MAIN_SOURCE;
            }
            else if (!secondary_valid) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            else if (now > internal.hysteresis_deadline) {
                return States::BOTH_SOURCES_VALID;
            }

            return States::MAIN_SOURCE_RECOVERING;
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

void RedundantRBSSelectorTask::updateHook()
{
    RedundantRBSSelectorTaskBase::updateHook();

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

    if (main_is_new_and_valid && secondary_is_new_and_valid) {
        PoseDivergence pose_divergence = checkDivergences(main_rbs, secondary_rbs);
        _pose_divergence.write(pose_divergence);
    }

    switch (state()) {
        case States::RUNNING:
            break;
        case States::BOTH_SOURCES_VALID:
            m_internal_state.hysteresis_deadline = base::Time();
            break;
        case States::MAIN_SOURCE_RECOVERING:
            if (m_internal_state.hysteresis_deadline.isNull()) {
                m_internal_state.hysteresis_deadline =
                    base::Time::now() + m_main_source_hysteresis;
            }
            break;
        case States::INVALID_MAIN_SOURCE:
            m_internal_state.hysteresis_deadline = base::Time();
            break;
        case States::INVALID_SECONDARY_SOURCE:
            m_internal_state.hysteresis_deadline = base::Time();
            break;
        case States::NO_VALID_SOURCES:
            break;
        default:
            throw std::invalid_argument("invalid controller state");
    }

    States next_state = updateState(state(), m_internal_state);
    if (next_state != state() && next_state != States::NO_VALID_SOURCES) {
        state(next_state);
    }

    if (next_state == States::NO_VALID_SOURCES) {
        return exception(States::NO_VALID_SOURCES);
    }

    if (next_state == States::BOTH_SOURCES_VALID ||
        next_state == States::INVALID_SECONDARY_SOURCE) {
        if (main_is_new_and_valid) {
            _rbs_out.write(main_rbs);
        }
    }
    else if (next_state == States::INVALID_MAIN_SOURCE ||
             next_state == States::MAIN_SOURCE_RECOVERING) {
        if (secondary_is_new_and_valid) {
            _rbs_out.write(secondary_rbs);
        }
    }
}

PoseDivergence RedundantRBSSelectorTask::checkDivergences(
    samples::RigidBodyState const& first,
    samples::RigidBodyState const& second)
{
    PoseDivergence divergence;
    divergence.time = base::Time::now();
    divergence.position_error_norm = (first.position - second.position).norm();
    divergence.position_divergent = divergence.position_error_norm > m_position_threshold;
    divergence.roll_error =
        base::Angle::fromRad(first.getRoll()) - base::Angle::fromRad(second.getRoll());
    divergence.pitch_error =
        base::Angle::fromRad(first.getPitch()) - base::Angle::fromRad(second.getPitch());
    divergence.yaw_error =
        base::Angle::fromRad(first.getYaw()) - base::Angle::fromRad(second.getYaw());
    divergence.roll_divergent =
        std::fabs(divergence.roll_error.getRad()) > m_angle_error_thresholds.roll;
    divergence.pitch_divergent =
        std::fabs(divergence.pitch_error.getRad()) > m_angle_error_thresholds.pitch;
    divergence.yaw_divergent =
        std::fabs(divergence.yaw_error.getRad()) > m_angle_error_thresholds.yaw;
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
