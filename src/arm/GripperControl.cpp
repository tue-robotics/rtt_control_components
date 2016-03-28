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

	/// Outports
	addPort("gripper_measurement",gripperMeasurementPort);
    addPort( "posout", posoutport );
    addPort( "velout", veloutport );
    addPort( "accout", accoutport );

	/// Properties
	addProperty( "threshold_closed", threshold_closed);
	addProperty( "max_pos", maxPos);
	addProperty( "min_pos", minPos);
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
	completed = true;
	desiredPos = 0.0;


	outpos.assign(1,0.0);
	outvel.assign(1,0.0);
	outacc.assign(1,0.0);
	
	return true;
}

bool GripperControl::startHook() 
{
    // Check validity of Ports:
    if ( !gripperCommandPort.connected() || !torqueInPort.connected() || !positionInPort.connected()) {
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
	// check for new commands
	if (gripperCommandPort.read(gripperCommand) == NewData){
		completed = false;
		if(gripperCommand.direction == tue_msgs::GripperCommand::OPEN) {
			log(Warning)<<"Grippercontrol: received gripper Command: Open"<<endlog();
			desiredPos = maxPos;
		}
		else {
			log(Warning)<<"Grippercontrol: received gripper Command: Close "<<endlog();
			desiredPos = minPos;
		}
	}
	
	// if active command 
	if (!completed){
		mRefPoint = mRefGenerator.generateReference(desiredPos, desiredVel, desiredAcc, InterpolDt, false, InterpolEps);
		outpos[0]=mRefPoint.pos;
		outvel[0]=mRefPoint.vel;
		outacc[0]=mRefPoint.acc;
		
		torqueInPort.read(torques);
		positionInPort.read(measPos);
		
		tue_msgs::GripperMeasurement gripperMeasurement;
		gripperMeasurement.direction = gripperCommand.direction;
		gripperMeasurement.torque = torques[GRIPPER_INDEX];
		gripperMeasurement.position = measPos[GRIPPER_INDEX];
		gripperMeasurement.end_position_reached = false;
		gripperMeasurement.max_torque_reached = false;

		if(gripperCommand.direction == tue_msgs::GripperCommand::OPEN){
			if (measPos[GRIPPER_INDEX] >= maxPos-0.02){
				log(Warning)<<"Gripper is OPEN - pos"<<endlog();
				gripperMeasurement.end_position_reached = true;
				gripperMeasurement.max_torque_reached = false;
				completed = true;
			}
		}
		else{
			// Closed on force requirement
			if (torques[GRIPPER_INDEX] >= threshold_closed) {
				log(Warning)<<"Gripper is CLOSED - force"<<endlog();
				gripperMeasurement.end_position_reached = false;
				gripperMeasurement.max_torque_reached = true;
				desiredPos = measPos[GRIPPER_INDEX];
				completed = true;
			}
			// Closed on Position requirement
			else if (measPos[GRIPPER_INDEX] <= minPos+0.2){
				log(Warning)<<"Gripper is CLOSED - pos "<<endlog();
				gripperMeasurement.end_position_reached = true;
				gripperMeasurement.max_torque_reached = false;
				desiredPos = measPos[GRIPPER_INDEX];
				completed = true;
			} 
		}
		
		gripperMeasurementPort.write(gripperMeasurement);
	} else {
		outpos[0]=desiredPos;
		outvel[0]=0.0;
		outacc[0]=0.0;
	}
	
	// Write outputs
	posoutport.write(outpos);
	veloutport.write(outvel);
	accoutport.write(outacc);
	
}

ORO_CREATE_COMPONENT(ARM::GripperControl)

