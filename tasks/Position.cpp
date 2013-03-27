/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Position.hpp"
#include <avalonmath.h>

using namespace raw_control_command_converter;

Position::Position(std::string const& name, TaskCore::TaskState initial_state)
    : PositionBase(name, initial_state)
{
	validPose = false;
}

Position::~Position()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Position.hpp for more detailed
// documentation about them.

// bool Position::configureHook()
// {
//     if (! PositionBase::configureHook())
//         return false;
//     return true;
// }
// bool Position::startHook()
// {
//     if (! PositionBase::startHook())
//         return false;
//     return true;
// }
void Position::updateHook()
{
    PositionBase::updateHook();

    controldev::RawCommand cmd;

    base::samples::RigidBodyState p;
    while(_pose_samples.read(p) == RTT::NewData){
    	pose = p;
    	if(!validPose){
		double heading,attitude,bank;
		Avalonmath::quaternionToEuler(pose.orientation,heading,attitude,bank);
		target_pose=p;
		target_heading = heading;
		validPose=true;
	}
    }

    if(!validPose){
	return;
    }

    while(_raw_command.read(cmd) == RTT::NewData){

    	base::AUVPositionCommand auv;
        base::LinearAngular6DCommand world;

	//TODO Add in this block handling of AUV motion commands in AUV Coordinate System and ann Delta Depth
	if(cmd.axisValue[0][0] != 0)
		target_pose.position[0] = pose.position[0] + (cmd.axisValue[0][0] * 2.0);
	if(cmd.axisValue[0][1] != 0)
		target_pose.position[1] = pose.position[1] + (cmd.axisValue[0][1] * 2.0);
	
	target_pose.position[2] =  0; //(cmd.axisValue[1][0] * -2.0) ; 
	
	target_pose.position = pose.orientation * target_pose.position;
	
	double heading,attitude,bank;
	Avalonmath::quaternionToEuler(pose.orientation,heading,attitude,bank);
	
	if(fabs(cmd.axisValue[0][2]) > 0.2)
		target_heading = heading - (cmd.axisValue[0][2] * (M_PI/2.0))/5.0;

	auv.x = target_pose.position[0];
        world.linear(0) = target_pose.position[0];
	auv.y = target_pose.position[1];
	world.linear(1) = target_pose.position[1];
	auv.z = (cmd.axisValue[1][0] * 2.0); //target_pose.position[2];
	world.linear(2) = (cmd.axisValue[1][0] * 2.0); //target_pose.position[2];
	world.linear(0) = 0;
        world.linear(1) = 0;
        auv.heading = target_heading;
        world.angular(2) = target_heading;
	printf("Converter: Current Position is: %f,%f,%f target: %f,%f,%f heading: %f\n",pose.position[0],pose.position[1],pose.position[2],auv.x,auv.y,auv.z,target_heading);
	_position_command.write(auv);
	_world_command.write(world);
    	
    }
}
// void Position::errorHook()
// {
//     PositionBase::errorHook();
// }
// void Position::stopHook()
// {
//     PositionBase::stopHook();
// }
// void Position::cleanupHook()
// {
//     PositionBase::cleanupHook();
// }

