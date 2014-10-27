/**
 * author: Bas Willems
 * email:  b.willems@student.tue.nl
 *
 * filename:             AddConstant.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AddConstant.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

AddConstant::AddConstant(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
	addProperty( "additions", addconstants ).doc("Float64 values to be added to the input of the component");

	// Adding ports
	addEventPort( "in", inport );
	addPort( "out", outport );
}
AddConstant::~AddConstant(){}

bool AddConstant::configureHook()
{

	return true;
}

bool AddConstant::startHook()
{

	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"The inputport is not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Outputport not connected!"<<endlog();
	}
	
	return true;
}

void AddConstant::updateHook()
{

	// Read the inputports
	doubles input(vectorsize,0.0);

	inport.read( input );

	// Calculate the output:
	doubles output(vectorsize,0.0);

	for ( uint i = 0; i < vectorsize; i++ )
	{
		output[i] = input[i] + addconstants[i];
	}

	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::AddConstant)
