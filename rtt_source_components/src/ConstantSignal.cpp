/** ConstantSignal.cpp
*
* @class ConstantSignal
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ConstantSignal.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

ConstantSignal::ConstantSignal(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{
  addProperty( "value", value ).doc("An array of double values, that determines the magnitude of the generated constant signal");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");
}

ConstantSignal::~ConstantSignal(){}

bool ConstantSignal::configureHook()
{
  Logger::In in("ConstantSignal::configureHook()");
  
  // Adding ports
  addPort( "out", outport ).doc("Vector of double values");

  Ts = getPeriod();
  
  return true;
}

bool ConstantSignal::startHook()
{
  Logger::In in("ConstantSignal::startHook()");
  
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

  return true;
}

void ConstantSignal::updateHook()
{
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i]=value[i];
  }
   
  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::ConstantSignal)
