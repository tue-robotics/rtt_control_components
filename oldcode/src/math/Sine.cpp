/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gain.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Sine.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Sine::Sine(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
	addProperty( "gain", gain ).doc("An array containing the gain values");

	// Adding ports
	addEventPort( "in", inport );
	addPort( "out", outport );
}
Sine::~Sine(){}

bool Sine::configureHook()
{
	return true;
}

bool Sine::startHook()
{
	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"Sine::inputport not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Sine::Outputport not connected!"<<endlog();
	}
	if ( vectorsize != gain.size() ) {
		log(Error)<<"Sine::vectorsizes is "<<vectorsize<<" while gain array is "<<gain.size()<<endlog();
		return false;
	}
	
	return true;
}

void Sine::updateHook()
{
	// Read the inputports
	doubles input(vectorsize,0.0);
	inport.read( input );

	// Calculate the output:
	doubles output(vectorsize,0.0);
	for ( uint i = 0; i < vectorsize; i++ ) {
		output[i] = gain[i] * sin(input[i]);
	}

	// Write the outputs
	outport.write( output );
}

void Sine::stopHook()
{
	// Close down neatly
	doubles output(vectorsize,0.0);
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Sine)
