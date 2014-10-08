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
	Logger::In in("Addition::Configure");	
	
	// Adding inports
	addEventPort( "in1_ev", inports[0]);
	for ( uint j = 1; j < numberofinputs; j++ ) {
		string name_inport = "in"+to_string(j+1);
		addPort( name_inport, inports[j]);
	}
	
	// Initialising input matrix inputs
	inputs.resize(numberofinputs);
	for ( uint j = 0; j < numberofinputs; j++ ) {
		inputs[j].assign(vectorsize,0.0);
	}
	
	return true;
}

bool Addition::startHook()
{
	Logger::In in("Addition::Start");	
	
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
	Logger::In in("Addition::Update");	
	
	// Read the inputports
	for ( uint j = 0; j < numberofinputs; j++ ) {
		doubles tempinput(vectorsize,0.0); 
		inports[j].read(tempinput);
		for ( uint i = 0; i < vectorsize; i++ ) {
			inputs[j][i] = tempinput[i];
		}
	}

	// Calculate the output:
	doubles output(vectorsize,0.0);
	for ( uint j = 0; j < numberofinputs; j++ ) {
		for ( uint i = 0; i < vectorsize; i++ ) {
			output[i] += inputs[j][i];
		}
	}
	
	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Addition)
