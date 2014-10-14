/**************************************************************************
 *                                                                        *
 *   S. Marinkov                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "GripperControl.hpp"

#define MAX_TORQUE 150.0

using namespace std;
using namespace RTT;
using namespace ARM;

GripperControl::GripperControl(const std::string& name) : TaskContext(name, PreOperational)
{
	/// Inports
	addPort("gripper_command", gripperCommandPort);
	addPort("torque_in", torqueInPort);
	addPort("position_in", positionInPort);
	addPort("reNullPort",reNullPort);
	addEventPort("resetGripperPort",resetGripperPort);

	/// Outports
	addPort("gripper_ref",gripperRefPort);
	addPort("gripper_measurement",gripperMeasurementPort);

	/// Properties
	addProperty( "threshold_closed", threshold_closed);
	addProperty( "gripper_gain", gripperGain);
	addProperty( "max_pos", maxPos);
}

GripperControl::~GripperControl() {}

bool GripperControl::configureHook() 
{
	Logger::In in("GripperControl::Configure");
	
	torques.assign(8,0.0); 
	measPos.assign(8,0.0);
	gripperPos.assign(1,0.0);
	completed = true;
	gripperHomed = false;
	
	return true;
}

bool GripperControl::startHook() 
{
	Logger::In in("GripperControl::Start");	
	
	return true;
}

void GripperControl::updateHook()
{
	Logger::In in("GripperControl::Update");
		
	bool resetGripper;
	
	if (resetGripperPort.read(resetGripper) == NewData){
		if(resetGripper){
			gripperPos[0]=0.0;
		}
	}
	
	if (gripperCommandPort.read(gripperCommand) == NewData){
		completed = false;
	}
		
	// Check whether supervisor specifies nulling of the relative encoders
	bool reNull;
	if(NewData == reNullPort.read(reNull)){
		if(reNull == true){
			log(Info)<<"Grippercontrol received reNull signal"<<endlog();
			// Increase threshold after the gripper has homed
			threshold_closed = threshold_closed*1.25;
			// Renull the gripperPos after homing
			gripperPos[0] = 0;
			gripperRefPort.write(gripperPos);
			// gripperHomed = true if all joints are homed
			gripperHomed = true;
		}
	}
			

	if (!completed){
		torqueInPort.read(torques);
		positionInPort.read(measPos);
		
		amigo_msgs::AmigoGripperMeasurement gripperMeasurement;
		gripperMeasurement.direction = gripperCommand.direction;
		gripperMeasurement.torque = torques[GRIPPER_INDEX];
		gripperMeasurement.position = measPos[GRIPPER_INDEX]/maxPos;
		gripperMeasurement.end_position_reached = false;
		gripperMeasurement.max_torque_reached = false;

		if(gripperCommand.direction == amigo_msgs::AmigoGripperCommand::OPEN){
			if (gripperPos[0] >= maxPos){
				log(Info)<<"Gripper is OPEN"<<endlog();
				gripperMeasurement.end_position_reached = true;
				completed = true;
			} 
			else{
				gripperPos[0] += gripperGain*PI/180;
			}
		} 
		else{
			//log(Warning)<<"gripper torques = "<<torques[GRIPPER_INDEX]<<endlog();
			if ( (torques[GRIPPER_INDEX] >= threshold_closed && torques[GRIPPER_INDEX] < MAX_TORQUE) || ( gripperHomed && (gripperPos[0] < 0.0)) ){
				log(Info)<<"Gripper is CLOSED"<<endlog();
				gripperMeasurement.end_position_reached = true;
				completed = true;
			} 
			else if(torques[GRIPPER_INDEX] < threshold_closed && torques[GRIPPER_INDEX] < MAX_TORQUE){
				//log(Warning)<<"GRIPPERCON: closing with torque = "<<torques[GRIPPER_INDEX]<<endlog();
				gripperPos[0] -= gripperGain*PI/180;
			}
			else {
				log(Error)<<"Gripper torque "<<torques[GRIPPER_INDEX]<<" exceeds maximum torque of "<<MAX_TORQUE<<" abort close_gripper"<<endlog();
				completed = true;
				gripperMeasurement.max_torque_reached = true;
			}
		}
		gripperRefPort.write(gripperPos);
		gripperMeasurementPort.write(gripperMeasurement);
	}
}

ORO_CREATE_COMPONENT(ARM::GripperControl)

