/** StepSignal.cpp
*
* @class StepSignal
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "StepSignal.hpp"

using namespace RTT;
using namespace SOURCES;

StepSignal::StepSignal(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{
 
  addProperty( "step_time", step_time );
  addProperty( "init_value", init_value );
  addProperty( "final_value", final_value );
  addProperty( "vector_size", vector_size );

  // Adding ports
  addPort( "out", outport );
}
StepSignal::~StepSignal(){}

bool StepSignal::configureHook()
{
  Ts = getPeriod();
  return true;
}

bool StepSignal::startHook()
{
  Logger::In in("StepSignal::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }
  
  for (uint i = 0; i < vector_size; i++) {
    if (step_time[i] <= 0) {
		log(Error)<<"StepSignal parameters not valid!"<<endlog();
		return false;
	}
  }
  
  if (vector_size < 1) {
    log(Error)<<"Size of the input vector must be positive!"<<endlog();
    return false;
  }

  if (Ts <= 0) {
    log(Error)<<"Sampling period not valid!"<<endlog();
    return false;
  }
  
  k = 0;

  return true;
}

void StepSignal::updateHook()
{
  Logger::In in("StepSignal::updateHook()");

  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	if (k*Ts < step_time[i]) {
		output[i] = init_value[i];
	} else {
		output[i] = final_value[i];
	}   
  }
  
  k++;
  
  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::StepSignal)
