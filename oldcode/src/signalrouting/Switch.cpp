/** Switch.hpp
 *
 * @class Switch
 *
 * \author Max Baeten
 * \date June, 2013
 * \version 1.0     Note that this component is not tested, and never used at this point 
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Switch.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALROUTING;

Switch::Switch(const string& name) :
    TaskContext(name, PreOperational),
    N(1)
{
  addProperty( "size_of_input", N ).doc("An unsigned integer that specifies the size of the doubles that are sent over the three ports. (same for all )");;
}

Switch::~Switch(){}

bool Switch::configureHook()
{
	
    log(Warning)<<"Switch: Start of configureHook()"<<endlog();
	addEventPort( "in_def", inport_default );
	addEventPort( "in_swi", inport_switched );
	addPort( "switchParam", inport_switchParam );
	addPort( "out", outport );
	
    switch_param = false;
    input_default.assign(N,0.0);
    input_switched.assign(N,0.0);
    output.assign(N,0.0);
    log(Warning)<<"Switch: End of configureHook()"<<endlog();
	return true;
}

bool Switch::startHook()
{

	if ( !inport_default.connected() ) {
		log(Error)<<"Switch: inport_default not connected!"<<endlog();
		return false;
	}

	if ( !inport_switched.connected() ) {
		log(Error)<<"Switch: inport_switched not connected!"<<endlog();
		return false;
	}

	if ( !outport.connected() ) {
		log(Error)<<"Switch: Outputport not connected!"<<endlog();
		return false;
	}

	return true;
}

void Switch::updateHook()
{

	inport_default.read(input_default);
	inport_switched.read(input_switched);

	if ( NewData == inport_switchParam.read(switch_param)) {
		log(Warning)<< "Switch: New SwitchingParam received" << endlog();
	}

	if (switch_param == false )	{
		output = input_default;
	}

	if (switch_param == true ) {
		output = input_switched;
	}

	outport.write( output );
}

ORO_CREATE_COMPONENT(SIGNALROUTING::Switch)
