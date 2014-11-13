/** Differentiate.cpp
*
* @class Differentiate
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Differentiate.hpp"

#define eps 1e-2

using namespace std;
using namespace RTT;
using namespace MATH;

Differentiate::Differentiate(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0),
		Ts(0.001),
		epsTs(0.0001),
		time_check(false)
{
  addProperty( "vector_size", vector_size );
  addProperty( "sampling_time", Ts );
  addProperty( "allowed_variation_Ts", epsTs );
  addProperty( "use_Ts_check", time_check );
}

Differentiate::~Differentiate(){}

bool Differentiate::configureHook()
{

	// Adding ports
	addEventPort( "in", inport );
	addPort( "out", outport );

	previous_input.resize(vector_size);
	previous_output.resize(vector_size);

	return true;
}

bool Differentiate::startHook()
{


	old_time = os::TimeService::Instance()->getNSecs()*1e-9;

	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}

	if ( !outport.connected() ) {
		log(Warning)<<"Outputport not connected!"<<endlog();
	}

	if (vector_size < 1) {
		log(Error)<<"Differentiate parameters not valid!"<<endlog();
		return false;
	}

	for (uint i = 0; i < vector_size; i++) {
		previous_input[i]  = 0.0;
		previous_output[i] = 0.0;
	}
	return true;
}

void Differentiate::updateHook()
{

 	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

	inport.read( input );

	determineDt();
	
	for (uint i = 0; i < vector_size; i++) {
		output[i] = (input[i] - previous_input[i])/dt;
	}
	
	if (time_check) {
		// user previous output if update time isn't reached
		if (dt < (Ts-epsTs) || dt > (Ts+epsTs)) {
			output = previous_output;
		} 
	}
	
	previous_input = input;
	previous_output = output;

	// Write the outputs
	outport.write( output );
}

void Differentiate::determineDt()
{
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
	dt = (new_time - old_time);
	old_time = new_time;
}

ORO_CREATE_COMPONENT(MATH::Differentiate)
