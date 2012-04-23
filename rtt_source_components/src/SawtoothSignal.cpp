/** SawtoothSignal.cpp
*
* @class SawtoothSignal
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SawtoothSignal.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

SawtoothSignal::SawtoothSignal(const string& name) : 
	TaskContext(name, PreOperational), 
		vector_size(0)
{
  
  addProperty( "slope", slope );
  addProperty( "period", period );
  addProperty( "vector_size", vector_size );

  // Adding ports
  addPort( "out", outport );
}

SawtoothSignal::~SawtoothSignal(){}

bool SawtoothSignal::configureHook()
{
  Ts = getPeriod();
  k.resize(vector_size);
  return true;
}

bool SawtoothSignal::startHook()
{
  Logger::In in("SawtoothSignal::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  for (uint i = 0; i < vector_size; i++) {
	if (period[i] <= 0) {
		log(Error)<<"SawtoothSignal parameters not valid!"<<endlog();
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

  return true;
}

void SawtoothSignal::updateHook()
{
  Logger::In in("SawtoothSignal::updateHook()");

  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	output[i] = slope[i]*k[i]*Ts;
	k[i]++;

	if (k[i]*Ts > (period[i])) {
	  k[i] = 0;
	}
  }

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::SawtoothSignal)
