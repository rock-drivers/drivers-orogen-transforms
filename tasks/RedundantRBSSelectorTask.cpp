/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RedundantRBSSelectorTask.hpp"

using namespace transforms;

bool rbsIsValid(base::samples::RigidBodyState const& rbs);

RedundantRBSSelectorTask::RedundantRBSSelectorTask(std::string const& name)
    : RedundantRBSSelectorTaskBase(name)
{
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
    return true;
}
bool RedundantRBSSelectorTask::startHook()
{
    if (!RedundantRBSSelectorTaskBase::startHook())
        return false;

    m_internal_state = InternalState();
    m_internal_state.main_source_deadline = m_internal_state.secondary_source_deadline =
        base::Time::now() + m_source_timeout;

    m_first_cycle = true;
    return true;
}

RedundantRBSSelectorTask::States updateState(
    RedundantRBSSelectorTask::States current_state,
    RedundantRBSSelectorTask::InternalState internal)
{
    // for readability's sake
    using States = RedundantRBSSelectorTask::States;
    base::Time now = base::Time::now();

    switch (current_state) {
        case States::BOTH_SOURCES_VALID:
            if (now > internal.main_source_deadline) {
                if (now > internal.secondary_source_deadline) {
                    return States::NO_VALID_SOURCES;
                }
                else {
                    return States::INVALID_MAIN_SOURCE;
                }
            }

            if (now > internal.secondary_source_deadline) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            return States::BOTH_SOURCES_VALID;
        case States::MAIN_SOURCE_RECOVERING:
            if (now > internal.main_source_deadline) {
                if (now > internal.secondary_source_deadline) {
                    return States::NO_VALID_SOURCES;
                }
                return States::INVALID_MAIN_SOURCE;
            }

            if (now < internal.hysteresis_deadline) {
                if (now > internal.secondary_source_deadline) {
                    return States::INVALID_SECONDARY_SOURCE;
                }
                return States::MAIN_SOURCE_RECOVERING;
            }

            if (now > internal.secondary_source_deadline) {
                return States::INVALID_SECONDARY_SOURCE;
            }
            return States::BOTH_SOURCES_VALID;
        case States::INVALID_MAIN_SOURCE:
            if (now > internal.secondary_source_deadline) {
                return States::NO_VALID_SOURCES;
            }
            if (now < internal.main_source_deadline) {
                return States::MAIN_SOURCE_RECOVERING;
            }
            return States::INVALID_MAIN_SOURCE;
        case States::INVALID_SECONDARY_SOURCE:
            if (now > internal.main_source_deadline) {
                return States::NO_VALID_SOURCES;
            }
            if (now < internal.secondary_source_deadline) {
                return States::BOTH_SOURCES_VALID;
            }
            return States::INVALID_SECONDARY_SOURCE;
        case States::NO_VALID_SOURCES:
            return States::NO_VALID_SOURCES;
        default:
            throw std::invalid_argument("invalid controller state");
    }
}

void RedundantRBSSelectorTask::updateHook()
{
    RedundantRBSSelectorTaskBase::updateHook();

    if (m_first_cycle) {
        m_first_cycle = false;
        state(States::BOTH_SOURCES_VALID);
    }

    base::Time now = base::Time::now();

    bool main_is_new_and_valid{false};
    base::samples::RigidBodyState main_rbs;
    if (_main_rbs_source.read(main_rbs) == RTT::NewData &&
        (main_is_new_and_valid = rbsIsValid(main_rbs))) {
        m_internal_state.main_source_deadline = now + m_source_timeout;
    }

    bool secondary_is_new_and_valid{false};
    base::samples::RigidBodyState secondary_rbs;
    if (_secondary_rbs_source.read(secondary_rbs) == RTT::NewData &&
        (secondary_is_new_and_valid = rbsIsValid(secondary_rbs))) {
        m_internal_state.secondary_source_deadline = now + m_source_timeout;
    }

    switch (state()) {
        case States::BOTH_SOURCES_VALID:
            m_internal_state.hysteresis_deadline = base::Time();
            break;
        case States::MAIN_SOURCE_RECOVERING:
            if (m_internal_state.hysteresis_deadline.isNull()) {
                m_internal_state.hysteresis_deadline = now + m_main_source_hysteresis;
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

    if (state() == States::BOTH_SOURCES_VALID ||
        state() == States::INVALID_SECONDARY_SOURCE) {
        if (main_is_new_and_valid) {
            _rbs_out.write(main_rbs);
        }
    }
    else if (secondary_is_new_and_valid) {
        _rbs_out.write(secondary_rbs);
    }
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

bool rbsIsValid(base::samples::RigidBodyState const& rbs)
{
    if (rbs.hasValidPosition() && rbs.hasValidVelocity() &&
        rbs.hasValidAngularVelocity()) {
        return true;
    }

    return false;
}
