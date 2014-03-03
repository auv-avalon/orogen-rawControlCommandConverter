/* Generated from orogen/lib/orogen/templates/tasks/Movement.cpp */

#include "Movement.hpp"

using namespace raw_control_command_converter;

Movement::Movement(std::string const& name, TaskCore::TaskState initial_state)
    : MovementBase(name, initial_state)
{
	initialized=false;
        heading_updated=false;
        depth=0;
        last_ground_position = -std::numeric_limits<double>::max();
        do_ground_following = false;
	target_depth=0;
	last_target_depth_valid = false;
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

    while(_orientation_readings.read(orientation) == RTT::NewData){
	if(!initialized){
		double heading = base::getYaw(orientation.orientation);
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

        double target_depth = cmd.axisValue[0][3] * _diveScale.get(); 
        if(_delta_depth_control.get()){
            if(target_depth != 0.0){
		target_depth += depth; 
	    	this->target_depth = target_depth;
		last_target_depth_valid=true;
	    }else{
		if(last_target_depth_valid){
			target_depth = depth;
			last_target_depth_valid=false;	
		}else{
			target_depth = this->target_depth;
		}
	    }
        }

        base::LinearAngular6DCommand world;
        world.time = base::Time::now();
        base::LinearAngular6DCommand world_depth;
        world_depth.time = base::Time::now();
        base::LinearAngular6DCommand aligned_velocity;
	auv.x_speed = cmd.axisValue[0][1];
        aligned_velocity.linear(0) = cmd.axisValue[0][1];
	auv.y_speed = cmd.axisValue[0][2] ;
        aligned_velocity.linear(1) = cmd.axisValue[0][2];

	world.angular(1) = cmd.axisValue[0][0];
	double heading = base::getYaw(orientation.orientation);
	if(!_do_ground_following){
            auv.z = target_depth;
            world.linear(2) = target_depth;
        }else{
            world_depth.linear(2) = last_ground_position + target_depth;
            if(last_ground_position == -std::numeric_limits<double>::max()){
                return error(SHOULD_DO_GROUND_FOLLOWING_WITHOUT_GROUND_DISTANCE);
            }
            auv.z = last_ground_position + target_depth;
        }
	
        if(fabs(cmd.axisValue[0][0]) > 0.2){
                heading_updated=true;
                target_heading = heading - (cmd.axisValue[0][0] * (M_PI/2.0))/_turnScale.get();
        }else if(heading_updated==true){
		target_heading = heading;
                heading_updated=false;
        }
        world.angular(0) = 0;
	auv.heading = target_heading;
	world.angular(2) = target_heading;
	while(world.angular(2) > M_PI)
            world.angular(2) -= 2*M_PI;
	while(world.angular(2) < -M_PI)
            world.angular(2) += 2*M_PI;
        
        _motion_command.write(auv);
        _world_command.write(world);
        _world_command_depth.write(world_depth);
        _aligned_velocity_command.write(aligned_velocity);
    	
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

