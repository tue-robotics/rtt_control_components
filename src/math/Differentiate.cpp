/** Differentiate.cpp
*
* @class Differentiate
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
* \Modified by Ton Peters
* \date November, 2014
* \version 1.1
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
		Ts(0),
		epsTs(0),
		time_check(false),
		discrete(false)
{
  addProperty( "vector_size", 			vector_size )	.doc("Size of the input vector.");
  addProperty( "sampling_time", 		Ts)				.doc("Optional: Sample time (used for time check and discrete calculation.");
  addProperty( "use_time_check", 			time_check)		.doc("Optional: Bool, use a time check on 2 consecutive updatehooks.");
  addProperty( "allowed_variation_Ts", 	epsTs) 			.doc("Optional: Ts +/- epsTs, used as time check.");
  addProperty( "use_discrete", 			discrete) 		.doc("Optional: Bool, se sampling time for calculation.");
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
	if ( (discrete || time_check) && Ts <= 0) {
		log(Error)<<"Differentiate, sampling_time not valid!"<<endlog();
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

	// use discrete or continuous time calculation (dt or Ts)
	double dt_used = dt;
	if (discrete) dt_used = Ts;

	// calculate output
	for (uint i = 0; i < vector_size; i++) {
		output[i] = (input[i] - previous_input[i])/dt_used;
	}

	// user previous output if update time isn't reached
	if (time_check) {
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
