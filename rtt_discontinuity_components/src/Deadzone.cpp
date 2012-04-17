/** Deadzone.cpp
*
* @class Deadzone
*
* \author Janno Lunenburg
* \date February, 2012
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "Deadzone.hpp"

using namespace std;
using namespace RTT;
using namespace DISCONTINUITIES;

Deadzone::Deadzone(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{

  addProperty( "start_deadzone", start_deadzone );
  addProperty( "end_deadzone", end_deadzone );
  addProperty( "vector_size", vector_size );
  
  addEventPort( "in", inport);
  addPort( "out", outport); 
   
}

Deadzone::~Deadzone(){}

bool Deadzone::configureHook()
{
   return true;
}

bool Deadzone::startHook()
{
  Logger::In in("Deadzone::startHook()");
  
  if ( !inport.connected() ) {
    log(Error)<<"Input port not connected!"<<endlog();
    return false;
  }

  if ( !outport.connected() ) {
	log(Warning)<<"Output port not connected!"<<endlog();
  }
 
  for (uint i = 0; i < vector_size; i++) {
	if (end_deadzone[i] < 0.0 || start_deadzone[i] > 0.0) {
		log(Error)<<"Parameters not valid!"<<endlog();
		return false;
	}
  }

  return true;
}

void Deadzone::updateHook()
{
  Logger::In in("Deadzone::updateHook()");

	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

  inport.read( input );
  
  output = input;
  for (uint i = 0; i < vector_size; i++) {
	  
	  if (input[i] <= start_deadzone[i]) {
		  output[i] = start_deadzone[i] + input[i];
	  }
	  else if (input[i] >= end_deadzone[i]) {
		  output[i] = end_deadzone[i] + input[i];
	  }
	  else if ( (input[i] > start_deadzone[i]) && (input[i] < end_deadzone[i]) ) {
		  output[i] = 0;
	  }
	  //log(Error)<<"Deadzone output"<<output[7]<<endlog();

  }
  
  outport.write( output );
}

ORO_CREATE_COMPONENT(DISCONTINUITIES::Deadzone)

