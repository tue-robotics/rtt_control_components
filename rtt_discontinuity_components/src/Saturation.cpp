/** Saturation.cpp
*
* @class Saturation
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Saturation.hpp"

using namespace std;
using namespace RTT;
using namespace DISCONTINUITIES;

Saturation::Saturation(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{

  addProperty( "upper_limit", upper_limit );
  addProperty( "lower_limit", lower_limit );
  addProperty( "vector_size", vector_size );
  
  addEventPort( "in", inport);
  addPort( "out", outport); 
   
}

Saturation::~Saturation(){}

bool Saturation::configureHook()
{
   return true;
}

bool Saturation::startHook()
{
  Logger::In in("Saturation::startHook()");
  
  if ( !inport.connected() ) {
    log(Error)<<"Input port not connected!"<<endlog();
    return false;
  }

  if ( !outport.connected() ) {
	log(Warning)<<"Output port not connected!"<<endlog();
  }
 
  for (uint i = 0; i < vector_size; i++) {
	if (upper_limit[i] < lower_limit[i]) {
		log(Error)<<"Parameters not valid!"<<endlog();
		return false;
	}
  }

  return true;
}

void Saturation::updateHook()
{
  Logger::In in("Saturation::updateHook()");

	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

  inport.read( input );
  
  output = input;
  for (uint i = 0; i < vector_size; i++) {
	  if (input[i] > upper_limit[i]) {
		  output[i] = upper_limit[i];
	  }
	  if (input[i] < lower_limit[i]) {
		  output[i] = lower_limit[i];
	  }
  }
  
  outport.write( output );
}

ORO_CREATE_COMPONENT(DISCONTINUITIES::Saturation)
