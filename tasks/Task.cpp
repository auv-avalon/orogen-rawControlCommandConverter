/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <avalonmath.h>

using namespace raw_control_command_converter;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }
// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }
void Task::updateHook()
{
    TaskBase::updateHook();

    controldev::RawCommand cmd;

    base::samples::RigidBodyState ori;
    while(_orientation_readings.read(ori) == RTT::NewData){
    	orientation = ori;
    }

    while(_raw_command.read(cmd) == RTT::NewData){
    	base::AUVMotionCommand auv;
	auv.x_speed = cmd.joyFwdBack * 2.0;
	auv.y_speed = cmd.joyLeftRight * 2.0;
	auv.z = cmd.joyThrottle * -2.0 ;
	double heading,attitude,bank;
	Avalonmath::quaternionToEuler(orientation.orientation,heading,attitude,bank);
	auv.heading = heading - (cmd.joyRotation * (M_PI/2.0));
	_motion_command.write(auv);
    	
    }
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

