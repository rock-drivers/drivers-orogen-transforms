/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RigidBodyStateAggregatorTask.hpp"

using namespace transforms;

RigidBodyStateAggregatorTask::RigidBodyStateAggregatorTask(std::string const& name)
    : RigidBodyStateAggregatorTaskBase(name)
{
}

RigidBodyStateAggregatorTask::~RigidBodyStateAggregatorTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RigidBodyStateAggregatorTask.hpp for more detailed
// documentation about them.

bool RigidBodyStateAggregatorTask::configureHook()
{
    if (!RigidBodyStateAggregatorTaskBase::configureHook())
        return false;
    return true;
}
bool RigidBodyStateAggregatorTask::startHook()
{
    if (!RigidBodyStateAggregatorTaskBase::startHook())
        return false;
    m_most_recent_pose_time = base::Time();
    return true;
}
void RigidBodyStateAggregatorTask::updateHook()
{
    RigidBodyStateAggregatorTaskBase::updateHook();

    base::samples::RigidBodyState sample;
    while (_source2ref.read(sample) == RTT::NewData) {
        if (sample.time <= m_most_recent_pose_time) {
            continue;
        }

        _source2ref_aggregated.write(sample);
        m_most_recent_pose_time = sample.time;
    }
}
void RigidBodyStateAggregatorTask::errorHook()
{
    RigidBodyStateAggregatorTaskBase::errorHook();
}
void RigidBodyStateAggregatorTask::stopHook()
{
    RigidBodyStateAggregatorTaskBase::stopHook();
}
void RigidBodyStateAggregatorTask::cleanupHook()
{
    RigidBodyStateAggregatorTaskBase::cleanupHook();
}
