/** Clock.cpp
*
* @class Clock
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <time.h>

#include "Clock.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

Clock::Clock(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");
}

Clock::~Clock(){}

bool Clock::configureHook()
{
  
  // Adding ports
  addPort( "out", outport ).doc("Vector of double values");

  return true;
}

bool Clock::startHook()
{
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1) {
    log(Error)<<"Clock parameters not valid!"<<endlog();
    return false;
  }

  start = clock();

  return true;
}

void Clock::updateHook()
{
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i]=(clock()-start)*1e-6;
  }
   
  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SOURCES::Clock)
