/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Addition.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Addition.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Addition::Addition(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
	addProperty( "numberofinputs", numberofinputs ).doc("An unsigned integer that specifies the number of inputs");

	// Adding outport
	addPort( "out", outport );
}
Addition::~Addition(){}

bool Addition::configureHook()
{
	// Adding inports
	addEventPort( "in1_ev", inports[0]);
	for ( uint j = 1; j < numberofinputs; j++ ) {
		string name_inport = "in"+to_string(j+1);
		addPort( name_inport, inports[j]);
	}
	
	return true;
}

bool Addition::startHook()
{
	
	// Check validity of Ports:
	for ( uint i = 1; i < numberofinputs; i++ ) {
		if (!inports[i].connected()) {
			log(Error)<<"Inputport[" << i << "] not connected!"<<endlog();
			return false;
		}
	}
	
	if ( !outport.connected() ) {
		log(Warning)<<"Outputport not connected!"<<endlog();
	}
	return true;
}

void Addition::updateHook()
{
	doubles output(vectorsize,0.0);
	// Read input ports
	for ( uint j = 0; j < numberofinputs; j++ ) {
		doubles tempinput(vectorsize,0.0); 
		inports[j].read(tempinput);
		for ( uint i = 0; i < vectorsize; i++ ) {
			output[i] += tempinput[i];
		}
	}
	
	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Addition)
