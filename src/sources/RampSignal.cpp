/** RampSignal.cpp
*
* @class RampSignal
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "RampSignal.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace SOURCES;

RampSignal::RampSignal(const string& name) : 
	TaskContext(name, PreOperational),
		slope(1.0), vector_size(1)
{
  addProperty( "slope", slope ).doc("An array of double values, that determines the slope of the generated ramp signal");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");
}

RampSignal::~RampSignal(){}

bool RampSignal::configureHook()
{
  Logger::In in("RampSignal::configureHook()");
  
  // Adding ports
  addPort( "out", outport ).doc("Vector of double values");

  Ts = getPeriod();
  
  return true;
}

bool RampSignal::startHook()
{
  Logger::In in("RampSignal::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1) {
    log(Error)<<"ConstantSignal parameters not valid!"<<endlog();
    return false;
  }

  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }
  
  k = 0;

  return true;
}

void RampSignal::updateHook()
{
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i]=k*slope[i]*Ts;
  }
  k++;
   
  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::RampSignal)
