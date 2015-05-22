#include "HandoverDetector.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

HandoverDetector::HandoverDetector(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "threshold", threshold).doc("Value to determine outer bounds");
	addProperty( "sendgrippergoal", sendgrippergoal).doc("Boolean valute to determine wether this orocos component should immediately call to open the gripper");
	
	// Port
	addPort("toggle", 			toggle_inport)			.doc("Toggle port");
	addPort("torques", 			torque_inport)			.doc("Torque in");
	addPort("result", 			result_outport)			.doc("Port to which the result is published");
	addPort("gripper", 			gripper_outport)		.doc("Port to send gripper command");
}

HandoverDetector::~HandoverDetector(){}

bool HandoverDetector::configureHook()
{
	toggle.data = false;
	toggled = false;
	lowerthreshold = 0.0;
	upperthreshold = 0.0;
	lowerthresholdreached = false;
	upperthresholdreached = false;
	
	return true;
}

bool HandoverDetector::startHook()
{
	// Check ports
	if (!torque_inport.connected()) {
		log(Warning)<<"HandoverDetector: Could not start Component: torques inport is not connected!"<<endlog();
		return false;
	}
	if (!toggle_inport.connected() || !result_outport.connected()) {
		log(Warning)<<"HandoverDetector: Could not start Component: toggle or result port not connected to ROS topic!"<<endlog();
		return false;
	}
	
	log(Warning)<<"HandoverDetector: Yeeeeeah! started HandoverDetector: !!!!!!!!"<<endlog();

	return true;
}

void HandoverDetector::updateHook()
{
	if (!toggled) {
		// Read topic in
		if (toggle_inport.read(toggle) == NewData) {
			
			if (toggle.data == true) {
				toggled = true;
				
				// read torque sensor of joint q1
				torque_inport.read(torques);
				log(Warning)<<"HandoverDetector: Toggled Handover Detector with starting value: " << torques[0] << "."<<endlog();
				
				lowerthreshold = (1.0-threshold)*torques[0];
				upperthreshold = (1.0+threshold)*torques[0];
			} else {
				log(Error)<<"HandoverDetector: Received incorrect data on toggle topic."<<endlog();
			}
		}
	} else {

		// Check Handover Criterium
		torque_inport.read(torques);
		
		if (torques[0] < lowerthreshold) {
			lowerthresholdreached = true;
			log(Warning)<<"HandoverDetector: Lower threshold reached"<<endlog();
		}
		if (torques[0] > upperthreshold) {
			upperthresholdreached = true;
			log(Warning)<<"HandoverDetector: Upper threshold reached"<<endlog();
		}
		if (lowerthresholdreached && upperthresholdreached) {
			HandoverDetected.data = true;
			result_outport.write(HandoverDetected);
			log(Warning)<<"HandoverDetector: Handover is detected. Both thresholds are reached"<<endlog();
			toggled = false;
			if (sendgrippergoal) {
				gripperCommand.direction = tue_msgs::GripperCommand::OPEN;
				gripper_outport.write(gripperCommand);
			}
		}
	}
	
	return;
}

ORO_CREATE_COMPONENT(ARM::HandoverDetector)
