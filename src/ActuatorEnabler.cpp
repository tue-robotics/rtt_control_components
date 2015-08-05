#include "ActuatorEnabler.hpp"

using namespace std;
using namespace RTT;
using namespace ACTUATORENABLER;

ActuatorEnabler::ActuatorEnabler(const string& name) : TaskContext(name, PreOperational), N_safeIn(1)
{
	//addPort( "safe_in",safe_inPort).doc("Receives safe = true if no errors are encountered");
	addPort( "enable",actuatorEnablePort).doc("boolean port to enable amplifier");
	addProperty( "ports_safe_in", N_safeIn).doc("number of safe inports");
}

ActuatorEnabler::~ActuatorEnabler(){}

bool ActuatorEnabler::configureHook()
{
	if ( N_safeIn > 0 ){
		for ( uint i = 0; i < N_safeIn; i++ ){
			if ( i==0 ){
				string portName = "safe_in"+ to_string(i+1);
				addPort( portName, safe_inPort[i] );
			}
			else {
				string portName = "safe_in"+ to_string(i+1);
				addEventPort( portName, safe_inPort[i] );
			}

		}
	}
	else {
		log(Error)<< "ActuatorEnabler:: ports_safe_in value invallid! "<<endlog();
		return false;
	}
	safe.assign(N_safeIn,false);
	ErrorWritten = false;
	return true;
}

bool ActuatorEnabler::startHook()
{ 		
	for ( uint i = 0; i < N_safeIn; i++ ){
		if (!safe_inPort[i].connected()) {
			log(Error)<<"ActuatorEnabler: safe_inPort " << i+1 << " not connected!"<<endlog();
			return false;
		}
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
	bool enable = true;

	// recieve new data
	for ( uint i = 0; i < N_safeIn; i++ ){
		bool value = false;
		if ( safe_inPort[i].read( value ) == NewData ){
			safe[i] = value;
		}
	}
	// check for disable signal
	for ( uint i = 0; i < N_safeIn; i++ ){
		if ( !safe[i] ){
			enable = false;
		}
	}
	// send enable signal
	actuatorEnablePort.write(enable);
}

ORO_CREATE_COMPONENT(ACTUATORENABLER::ActuatorEnabler)
