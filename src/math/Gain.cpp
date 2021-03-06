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

#include "Gain.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Gain::Gain(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
	addProperty( "gain", gain ).doc("An array containing the gain values");

	// Adding ports
	addEventPort( "in", inport );
	addPort( "out", outport );
}
Gain::~Gain(){}

bool Gain::configureHook()
{
	return true;
}

bool Gain::startHook()
{
	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"Gain::inputport not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Gain::Outputport not connected!"<<endlog();
	}
	if ( vectorsize != gain.size() ) {
		log(Error)<<"Gain::vectorsizes is "<<vectorsize<<" while gain array is "<<gain.size()<<endlog();
		return false;
	}
	
	return true;
}

void Gain::updateHook()
{
	// Read the inputports
	doubles input(vectorsize,0.0);
	inport.read( input );

	// Calculate the output:
	doubles output(vectorsize,0.0);
	for ( uint i = 0; i < vectorsize; i++ ) {
		output[i] = gain[i] * input[i];
	}

	// Write the outputs
	outport.write( output );
}

void Gain::stopHook()
{
	// Close down neatly
	doubles output(vectorsize,0.0);
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Gain)
