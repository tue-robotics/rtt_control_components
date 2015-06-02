#include "HandoverDetector.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

HandoverDetector::HandoverDetector(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "threshold", 		threshold).doc("Value to determine outer bounds");
	
	// Port
	addPort("toggle_human2robot", 	toggle_h2r_inport)		.doc("Toggle port for human to robot handover");
	addPort("toggle_robot2human", 	toggle_r2h_inport)		.doc("Toggle port for robot to human handover");
	addPort("torques", 				torque_inport)			.doc("Torque in");
	addPort("result", 				result_outport)			.doc("Port to which the result is published");
	addPort("gripper", 				gripper_outport)		.doc("Port to send gripper command");
}

HandoverDetector::~HandoverDetector(){}

bool HandoverDetector::configureHook()
{
	toggle_msg.data = false;
	toggled_r2h = false;
	toggled_h2r = false;
	lowerthreshold = 0.0;
	upperthreshold = 0.0;
	sign = 1.0;
	
	return true;
}

bool HandoverDetector::startHook()
{
	// Check ports
	if (!torque_inport.connected()) {
		log(Warning)<<"HandoverDetector: Could not start Component: torques inport is not connected!"<<endlog();
		return false;
	}
	if (!toggle_h2r_inport.connected() || !toggle_r2h_inport.connected() || !result_outport.connected()) {
		log(Warning)<<"HandoverDetector: Could not start Component: toggle or result port not connected to ROS topic!"<<endlog();
		return false;
	}

	return true;
}

void HandoverDetector::updateHook()
{
	if (!toggled_r2h && !toggled_h2r ) {	
		
		// Read toggle topics	
		if (toggle_r2h_inport.read(toggle_msg) == NewData) { 			// Read Robot 2 Human topic
			toggled_r2h = true;
			log(Warning)<<"HandoverDetector: Toggled Handover Detector: Robot 2 Human"<<endlog();
		} else if (toggle_h2r_inport.read(toggle_msg) == NewData) {		// Read Human 2 Robot topic
			toggled_h2r = true;
			log(Warning)<<"HandoverDetector: Toggled Handover Detector: Human 2 Robot"<<endlog();
		}
		
		// If toggled, read torque and calculate thresholds
		if (toggled_r2h == true || toggled_h2r == true) {
			if (toggle_msg.data == true) {
				
				// read torque sensor of joint q1
				torque_inport.read(torques);
								
				sign = (torques[0]/(sqrt(torques[0]*torques[0])));			
				lowerthreshold = (1.0-sign*threshold)*torques[0];
				upperthreshold = (1.0+sign*threshold)*torques[0];				
				
				log(Warning)<<"HandoverDetector: [Lowerbound < measured < Upperbound] -> [" << lowerthreshold  << "<" << torques[0]  << "<" << upperthreshold<< "] \n \n"<<endlog();
				
			} else {
				log(Error)<<"HandoverDetector: Received incorrect data on toggle topic."<<endlog();
			}			
		}
	} else {

		// Check Handover Criterium
		torque_inport.read(torques);		
		if (torques[0] < lowerthreshold || torques[0] > upperthreshold) {	
			
			log(Warning)<<"HandoverDetector: Detected handover since [Lowerbound < measured < Upperbound] -> [" << lowerthreshold  << "<" << torques[0]  << "<" << upperthreshold<< "] \n \n"<<endlog();
									
			// Send Gripper command
			if (toggled_r2h) {
				gripperCommand_msg.direction = tue_msgs::GripperCommand::OPEN;
			} else if (toggled_h2r) {
				gripperCommand_msg.direction = tue_msgs::GripperCommand::CLOSE;
			}
			gripper_outport.write(gripperCommand_msg);
			
			// Sending output
			result_outport.write(toggle_msg);
			

			// Resetting parameters
			this->configureHook();
		}
	}
	
	return;
}

ORO_CREATE_COMPONENT(ARM::HandoverDetector)
