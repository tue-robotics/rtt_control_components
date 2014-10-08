/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Subtraction.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Subtraction.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Subtraction::Subtraction(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");;

	// Adding ports
	addPort( "in_plus", inport_plus );
	addEventPort( "in_minus", inport_minus );
	addPort( "out", outport );
}
Subtraction::~Subtraction(){}

bool Subtraction::configureHook()
{
	Logger::In in("Subtraction::Configure");	
	
	return true;
}

bool Subtraction::startHook()
{
	Logger::In in("Subtraction::Start");
	
	// Check validity of Ports:
	if ( !inport_plus.connected() || !inport_minus.connected() ) {
		log(Error)<<"One or more inputports not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Outputport not connected!"<<endlog();
	}
	return true;
}

void Subtraction::updateHook()
{
	Logger::In in("Subtraction::Update");	
	
	// Read the inputports
	doubles input_plus(vectorsize,0.0);
	doubles input_minus(vectorsize,0.0);

	inport_plus.read( input_plus );
	inport_minus.read( input_minus );

	// Calculate the output:
	doubles output(vectorsize,0.0);
	for ( uint i = 0; i < vectorsize; i++ ) {
		output[i] = input_plus[i] - input_minus[i];
	}

	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Subtraction)
