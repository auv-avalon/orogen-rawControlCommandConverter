/* Generated from orogen/lib/orogen/templates/tasks/Movement.cpp */

#include "Movement.hpp"
#include <avalonmath.h>

using namespace raw_control_command_converter;

Movement::Movement(std::string const& name, TaskCore::TaskState initial_state)
    : MovementBase(name, initial_state)
{
	initialized=false;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Movement.hpp for more detailed
// documentation about them.

// bool Movement::configureHook()
// {
//     if (! MovementBase::configureHook())
//         return false;
//     return true;
// }
// bool Movement::startHook()
// {
//     if (! MovementBase::startHook())
//         return false;
//     return true;
// }
void Movement::updateHook()
{
    MovementBase::updateHook();

    controldev::RawCommand cmd;

    base::samples::RigidBodyState ori;
    while(_orientation_readings.read(ori) == RTT::NewData){
    	orientation = ori;
	if(!initialized){
		double heading,attitude,bank;
		Avalonmath::quaternionToEuler(orientation.orientation,heading,attitude,bank);
		target_heading = heading;
		initialized=true;
	}
    }
	
    if(!initialized)
    	return;

    while(_raw_command.read(cmd) == RTT::NewData){
    	base::AUVMotionCommand auv;
	auv.x_speed = cmd.joyFwdBack * 2.0;
	auv.y_speed = cmd.joyLeftRight * 2.0;
	auv.z = cmd.joyThrottle * -2.0 ;
	double heading,attitude,bank;
	Avalonmath::quaternionToEuler(orientation.orientation,heading,attitude,bank);
	if(cmd.joyRotation != 0)
		target_heading = heading - (cmd.joyRotation * (M_PI/2.0));
	
	auv.heading = target_heading;
	_motion_command.write(auv);
    	
    }
}
// void Movement::errorHook()
// {
//     MovementBase::errorHook();
// }
// void Movement::stopHook()
// {
//     MovementBase::stopHook();
// }
// void Movement::cleanupHook()
// {
//     MovementBase::cleanupHook();
// }

