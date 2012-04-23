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
		vector_size(0)
{
  addProperty( "vector_size", vector_size );
}

Differentiate::~Differentiate(){}

bool Differentiate::configureHook()
{
  Logger::In in("Differentiate::configureHook()");

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );

  previous_input.resize(vector_size);
  
  return true;
}

bool Differentiate::startHook()
{
  Logger::In in("Differentiate::startHook()");
  
  old_time = os::TimeService::Instance()->getNSecs()*1e-9;

  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
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
  }
  
  return true;
}

void Differentiate::updateHook()
{
	Logger::In in("Differentiate::updateHook()");

 	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

	inport.read( input );

	determineDt();

	for (uint i = 0; i < vector_size; i++) {
			output[i] = (input[i] - previous_input[i])/dt;
	}
	previous_input = input;

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
