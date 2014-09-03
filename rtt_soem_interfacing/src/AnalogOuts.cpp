#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogOuts.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogOuts::AnalogOuts(const string& name) : TaskContext(name, PreOperational)
{
  addPort( "Analog_out", Analog_out_port );
  addEventPort( "wheels", wheels_port );
  addEventPort( "spindle", spindle_port );
}
AnalogOuts::~AnalogOuts(){}

bool AnalogOuts::configureHook()
{
	log(Error) << "AnalogOuts: DEPRECATED COMPONENT. Use AnalogOutsGeneric" << endlog();
	
	amsg.values.assign(8,0.0);
	amsg.values[0] =  0.008;
	amsg.values[3] = -0.008; // Leakage compensation
	output.assign(8,0.0);
	
	return true;
}

bool AnalogOuts::startHook()
{
	return true;
}

void AnalogOuts::updateHook()
{
	if ( NewData == wheels_port.read(wheels)){
		for ( uint i = 0; i < 4; i++ ) {
			output[i] = wheels[i];
		}
	}
	
	if ( NewData == spindle_port.read(spindle)) {
		output[4] = spindle[0];
	}
	
	for ( uint i = 0; i < 8; i++ ) {
		amsg.values[i] = output[i];
	}
	
	Analog_out_port.write(amsg);
}

ORO_CREATE_COMPONENT(SOEM::AnalogOuts)
