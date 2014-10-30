#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "ActuatorEnabler.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

ActuatorEnabler::ActuatorEnabler(const string& name) : TaskContext(name, PreOperational)
{
    addPort( "safe_in",safe_inPort).doc("Receives safe = true if no errors are encountered");
    addPort( "actuator_enable",actuatorEnablePort).doc("boolean port to enable amplifier");
}

ActuatorEnabler::~ActuatorEnabler(){}

bool ActuatorEnabler::configureHook()
{  
	safe = false;
	ErrorWritten = false;
	return true;
}

bool ActuatorEnabler::startHook()
{ 		
	if (!safe_inPort.connected()) {
		log(Error)<<"ActuatorEnabler: safe_inPort not connected!"<<endlog();
		return false;
	}
	if (!actuatorEnablePort.connected()) {
		log(Error)<<"ActuatorEnabler: actuatorEnablePort not connected!"<<endlog();
		return false;
	}
	
	TimeLastSafeReceived = 0.0;
	
	return true;
}

void ActuatorEnabler::updateHook()
{
	if ( safe_inPort.read(safe) == NewData ) {
		TimeLastSafeReceived = os::TimeService::Instance()->getNSecs()*1e-9;
		if (safe) {
			actuatorEnablePort.write(true);
			ErrorWritten = false;
		}
		else {
			actuatorEnablePort.write(false);
			if (ErrorWritten) {
				log(Error)<<"ActuatorEnabler: Safe = false received. Disabling Actuator!"<<endlog();	
				ErrorWritten = true;
			}
		}
	}
	else {
		long double TimeNow = os::TimeService::Instance()->getNSecs()*1e-9;
		if ((TimeNow - TimeLastSafeReceived > 0.01) ) {
			actuatorEnablePort.write(false);
		}
	}
}

ORO_CREATE_COMPONENT(SUPERVISORY::ActuatorEnabler)
