/** UniformRandomNumber.cpp
*
* @class UniformRandomNumber
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/
 
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "UniformRandomNumber.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

UniformRandomNumber::UniformRandomNumber(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{

  /*** Reading properties ***/
  addProperty( "maximum", maximum ).doc("An array of double values that determines the maximum value of the generated signals.");
  addProperty( "minimum", minimum ).doc("An array of double values that determines the minimum value of the generated signals.");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");

  /*** Adding ports ***/
  addPort( "out", outport );
  
}

UniformRandomNumber::~UniformRandomNumber(){}

bool UniformRandomNumber::configureHook()
{
  Logger::In in("UniformRandomNumber::configureHook()");
  
  Ts = getPeriod();
  return true;
}

bool UniformRandomNumber::startHook()
{
  Logger::In in("UniformRandomNumber::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1) {
    log(Error)<<"UniformRandomNumber parameters not valid!"<<endlog();
    return false;
  }

  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }

  for ( uint i = 0; i < vector_size; i++ ) {
	  if (maximum[i] < 0.0 || minimum[i] > 0.0) {
		  log(Error)<<"UniformRandomNumber parameters not valid!"<<endlog();
		  return false;
	  }
  }
  
  return true;
}

void UniformRandomNumber::updateHook()
{
  Logger::In in("UniformRandomNumber::updateHook()");

  doubles output(vector_size,0.0);

  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i] = randomnumgen(maximum[i],minimum[i]);
  }
  
  /*** Write the outputs ***/
  outport.write( output );
}

double UniformRandomNumber::randomnumgen(double low, double high)
/* uniform random variate generator */
/* minimum low, maximum high */
{
  double range=(high-low);
  double num = rand() * range / RAND_MAX + low ;
  return(num);
}

ORO_CREATE_COMPONENT(SOURCES::UniformRandomNumber)
