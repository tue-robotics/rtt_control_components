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
	toggled_time = 0;
	
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
				log(Warning)<<"HandoverDetector: Toggled Handover Detector"<<endlog();
			}
		}
	} else {
		toggled_time++;
		if (toggled_time >= 60) {
			std_msgs::Bool HandoverDetected;
			HandoverDetected.data = true;
			result_outport.write(HandoverDetected);
			log(Warning)<<"HandoverDetector: Handover Detected"<<endlog();
		}
	}
	
	return;
}

ORO_CREATE_COMPONENT(ARM::HandoverDetector)
