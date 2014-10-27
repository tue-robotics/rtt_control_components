/** Demux.cpp
*
* @class Demux
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Demux.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALROUTING;

Demux::Demux(const string& name) : 
	TaskContext(name, PreOperational),
		N(2)
{
  addProperty( "number_of_outputs", N );
}

Demux::~Demux(){}

bool Demux::configureHook()
{
	
	addEventPort( "in", inport);
	
	// Creating ports
	for ( uint i = 0; i < N; i++ )
	{
		string name_outport = "out"+to_string(i+1);
		addPort( name_outport, outports[i] );
	}
	
	return true;
}

bool Demux::startHook()
{

	if ( !inport.connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}

	if ( N > maxN ) {
		log(Error)<<"Max number of ports exceeded!"<<endlog();
		return false;
	}

	if ( N < 1 ) {
		log(Error)<<"Number of ports must be at least 1!"<<endlog();
		return false;
	}

	// Check validity of Ports:
	for (uint i = 0; i < N; i++) {
		if ( !outports[i].connected() ) {
			log(Warning)<<"Output port "<< i+1 <<" not connected!"<<endlog();
		}
	}

	return true;
}

void Demux::updateHook()
{

	doubles input(N,0.0);

	inport.read( input );
	for ( uint i = 0; i < N; i++ ) {
		outports[i].write( input[i] );
	}
}

ORO_CREATE_COMPONENT(SIGNALROUTING::Demux)
