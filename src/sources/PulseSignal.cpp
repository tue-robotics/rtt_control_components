/** PulseSignal.cpp
 *
 * @class PulseSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PulseSignal.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

PulseSignal::PulseSignal(const string& name) : 
	    TaskContext(name, PreOperational),
			vector_size(0)
{

  addProperty( "amplitude", amplitude );
  addProperty( "period", period );
  addProperty( "pulse_width", pulse_width );
  addProperty( "phase_delay", phase_delay );
  addProperty( "vector_size", vector_size );

  // Adding ports
  addPort( "out", outport );
}

PulseSignal::~PulseSignal(){}

bool PulseSignal::configureHook()
{
  Ts = getPeriod();
  k.resize(vector_size);
  return true;
}

bool PulseSignal::startHook()
{

  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  for (uint i = 0; i < vector_size; i++) {
	if (amplitude[i] < 0.0 || period[i] <= 0 || pulse_width[i] <= 0) {
		log(Error)<<"PulseSignal parameters not valid!"<<endlog();
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
  
  for (uint i = 0; i < vector_size; i++) {
	  k[i] = 0;
  }

  return true;
}

void PulseSignal::updateHook()
{
  
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  if (k[i]*Ts >= phase_delay[i] && k[i]*Ts < (phase_delay[i] + pulse_width[i]) ) {
		output[i] = amplitude[i];
	  } else {
		output[i] = 0.0;
	  }   
	  k[i]++;

	  if (k[i]*Ts >= (period[i])) {
		k[i] = 0;
	  }
  }
  
  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::PulseSignal)
