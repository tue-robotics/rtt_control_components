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
	addPort("gripper_measurement",gripperMeasurementPort);
    addPort( "posout", posoutport );
    addPort( "velout", veloutport );
    addPort( "accout", accoutport );

	/// Properties
	addProperty( "threshold_closed", threshold_closed);
	addProperty( "gripper_gain", gripperGain);
	addProperty( "max_pos", maxPos);
    addProperty( "gripper_vel", desiredVel);
    addProperty( "gripper_acc", desiredAcc);
    addProperty( "InterpolDt", InterpolDt);
    addProperty( "InterpolEps", InterpolEps);
}

GripperControl::~GripperControl() {}

bool GripperControl::configureHook() 
{	
	torques.assign(8,0.0); 
	measPos.assign(8,0.0);
	gripperPos.assign(1,0.0);
	completed = true;
	gripperHomed = false;
	
	return true;
}

bool GripperControl::startHook() 
{
    // Check validity of Ports:
    if ( !gripperCommandPort.connected() || !torqueInPort.connected() || !positionInPort.connected()) {
        log(Warning)<<"GripperControl: Could not start component: one of the gripperCommandPort, torqueInPort, positionInPort is not connected!"<<endlog();
        return false;
    }
    // Check validity of Ports:
    if ( !reNullPort.connected() || !resetGripperPort.connected() ) {
        log(Warning)<<"GripperControl: Could not start component: one of the gripperCommandPort, torqueInPort, positionInPort is not connected!"<<endlog();
        return false;
    }
    if ( !posoutport.connected() && !veloutport.connected() && !accoutport.connected()) {
        log(Warning)<<"GripperControl: Could not start component: none of the posoutport, veloutport, accoutport is connected!"<<endlog();
        return false;
    }
    if ( !gripperMeasurementPort.connected() ) {
        log(Warning)<<"GripperControl: gripperMeasurementPort is not connected!"<<endlog();
    }

    //Set the starting value to the current actual value
    positionInPort.read(measPos);
    mRefGenerator.setRefGen(measPos[GRIPPER_INDEX]);

	return true;
}

void GripperControl::updateHook()
{
	bool resetGripper;
    doubles outpos(1,0.0);
    doubles outvel(1,0.0);
    doubles outacc(1,0.0);
    double desiredPos = 0.0;
	
	if (resetGripperPort.read(resetGripper) == NewData){
		if(resetGripper){
			gripperPos[0]=0.0;
		}
	}
	
	if (gripperCommandPort.read(gripperCommand) == NewData){
		completed = false;
        if(gripperCommand.direction == tue_msgs::GripperCommand::OPEN) {
            desiredPos = 25.0;
        }
        else {
            desiredPos = -25.0;
        }
	}
		
	// Check whether supervisor specifies nulling of the relative encoders
	bool reNull;
	if(NewData == reNullPort.read(reNull)){
		if(reNull == true){
			log(Info)<<"Grippercontrol received reNull signal"<<endlog();
			// Increase threshold after the gripper has homed
			threshold_closed = threshold_closed*1.25;
			// Renull the gripperPos after homing
            desiredPos = 0.0;

			// gripperHomed = true if all joints are homed
			gripperHomed = true;
		}
	}

    mRefPoint = mRefGenerator.generateReference(desiredPos, desiredVel, desiredAcc, InterpolDt, false, InterpolEps);
    outpos[0]=mRefPoint.pos;
    outvel[0]=mRefPoint.vel;
    outacc[0]=mRefPoint.acc;

	if (!completed){
		torqueInPort.read(torques);
		positionInPort.read(measPos);
		
		tue_msgs::GripperMeasurement gripperMeasurement;
		gripperMeasurement.direction = gripperCommand.direction;
		gripperMeasurement.torque = torques[GRIPPER_INDEX];
		gripperMeasurement.position = measPos[GRIPPER_INDEX]/maxPos;
		gripperMeasurement.end_position_reached = false;
		gripperMeasurement.max_torque_reached = false;

		if(gripperCommand.direction == tue_msgs::GripperCommand::OPEN){
			if (gripperPos[0] >= maxPos){
				log(Info)<<"Gripper is OPEN"<<endlog();
				gripperMeasurement.end_position_reached = true;
				completed = true;
			} 
		} 
		else{
			//log(Warning)<<"gripper torques = "<<torques[GRIPPER_INDEX]<<endlog();
			if ( (torques[GRIPPER_INDEX] >= threshold_closed && torques[GRIPPER_INDEX] < MAX_TORQUE) || ( gripperHomed && (gripperPos[0] < 0.0)) ){
				log(Info)<<"Gripper is CLOSED"<<endlog();
				gripperMeasurement.end_position_reached = true;
				completed = true;
			} 
			else {
				log(Error)<<"Gripper torque "<<torques[GRIPPER_INDEX]<<" exceeds maximum torque of "<<MAX_TORQUE<<" abort close_gripper"<<endlog();
				completed = true;
				gripperMeasurement.max_torque_reached = true;
			}
		}

        // Write outputs
        posoutport.write(outpos);
        veloutport.write(outvel);
        accoutport.write(outacc);
		gripperMeasurementPort.write(gripperMeasurement);
	}
}

ORO_CREATE_COMPONENT(ARM::GripperControl)

