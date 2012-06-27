/* Generated from orogen/lib/orogen/templates/tasks/Movement.cpp */

#include "Movement.hpp"
#include <avalonmath.h>

using namespace raw_control_command_converter;

Movement::Movement(std::string const& name, TaskCore::TaskState initial_state)
    : MovementBase(name, initial_state)
{
	initialized=false;
        heading_updated=false;
        depth=0;
        last_ground_position = -std::numeric_limits<double>::max();
        do_ground_following = false;
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
                if(!orientation.hasValidPosition(2)){
                    return error(GOT_POSE_WITHOUT_DEPTH);
                }
                depth=orientation.position[2];
		initialized=true;
	}
    }
	
    if(!initialized)
    	return;
    {
    base::samples::RigidBodyState rbs;
    while(_ground_distance.read(rbs) == RTT::NewData){
        if(rbs.position.z() != 0.0)
            last_ground_position = depth - rbs.position.z();
    }
    }

    if(_raw_command.read(cmd) != RTT::NoData){
    	base::AUVMotionCommand auv;
	auv.x_speed = cmd.joyFwdBack;
	auv.y_speed = -cmd.joyLeftRight ;
	double heading,attitude,bank;
	Avalonmath::quaternionToEuler(orientation.orientation,heading,attitude,bank);
        if(cmd.additionalAxis[1] == -1){
            do_ground_following = true;
        }else if(cmd.additionalAxis[1] == 1){
            do_ground_following = false;
        }
	if(!do_ground_following){
            auv.z = cmd.joyThrottle * _diveScale.get() ;
        }else{
            if(last_ground_position == -std::numeric_limits<double>::max()){
                return error(SHOULD_DO_GROUND_FOLLOWING_WITHOUT_GROUND_DISTANCE);
            }
            auv.z = last_ground_position + (cmd.joyThrottle * _diveScale.get()) ;
        }
	
        if(fabs(cmd.joyRotation) > 0.2){
                heading_updated=true;
                target_heading = heading - (cmd.joyRotation * (M_PI/2.0))/_turnScale.get();
        }else if(heading_updated==true){
		target_heading = heading;
                heading_updated=false;
        }
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

