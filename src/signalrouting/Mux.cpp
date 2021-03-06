/** Mux.cpp
*
* @class Mux
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Mux.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALROUTING;

Mux::Mux(const string& name) : 
	TaskContext(name, PreOperational),
		N(2)
{
  addProperty( "number_of_inputs", N ).doc("An unsigned integer that specifies the number of input ports");;
}

Mux::~Mux(){}

bool Mux::configureHook()
{

	for ( uint i = 0; i < N; i++ ) {
		string name_inport = "in"+to_string(i+1);
		addEventPort( name_inport, inports[i] );
	}
	
	addPort( "out", outport );
	
	return true;
}

bool Mux::startHook()
{

	if ( N > maxN ) {
		log(Error)<<"Max number of ports exceeded!"<<endlog();
		return false;
	}

	if ( N < 1 ) {
		log(Error)<<"Number of ports must be at least 1!"<<endlog();
		return false;
	}

	// Check validity of Ports
	for (uint i = 0; i < N; i++) {
		if ( !inports[i].connected() ) {
			log(Error)<<"Input port "<< i+1 <<" not connected!"<<endlog();
			return false;
		}
	}

	if ( !outport.connected() ) {
		log(Error)<<"Outputport not connected!"<<endlog();
		return false;
	}

	return true;
}

void Mux::updateHook()
{
	
	doubles output(N,0.0);  

	for ( uint i = 0; i < N; i++ ) {
		inports[i].read( output[i] );
	}

	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(SIGNALROUTING::Mux)
