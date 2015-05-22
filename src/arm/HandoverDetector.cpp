#include "HandoverDetector.hpp"

using namespace std;
using namespace RTT;
using namespace ARM;

HandoverDetector::HandoverDetector(const string& name) : TaskContext(name, PreOperational)
{
	// Port
	addPort("toggle", 			toggle_inport)			.doc("Toggle port");
	addPort("error", 			error_inport)			.doc("Control error in");
	addPort("controloutput", 	controloutput_inport)	.doc("Control output in");
	addPort("torque", 			torque_inport)			.doc("Torque in");
	addPort("result", 			result_outport)			.doc("Port to which the result is published");
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
	if (!error_inport.connected() || !controloutput_inport.connected() || !torque_inport.connected()) {
		log(Warning)<<"HandoverDetector: None of the input ports is connected Error_inport not connected!"<<endlog();
	}
	if (!toggle_inport.connected() || !result_outport.connected()) {
		log(Warning)<<"HandoverDetector: Could not start Component: toggle or result port not connected to ROS topic!"<<endlog();
		return false;
	}

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
				
				lowerthreshold = 0.925*torques[0];
				upperthreshold = 1.075*torques[0];
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
		}
	}
	
	return;
}

ORO_CREATE_COMPONENT(ARM::HandoverDetector)
