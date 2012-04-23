/** Equals.cpp
 *
 * @class Equals
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Equals.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Equals::Equals(const string& name) :
	    TaskContext(name, PreOperational)
{
  // Adding property
  addProperty( "values", values );
  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}

Equals::~Equals(){}

bool Equals::configureHook()
{
  return true;
}

bool Equals::startHook()
{
  Logger::In in("Equals::startHook()");

  // Check validity of Ports
  if ( !inport.connected() ) {
    log(Error)<<"Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }

  if ( values.size() == 0 ) {
    log(Error)<<"Property values not valid!"<<endlog();
    return false;
  }

  return true;
}

void Equals::updateHook()
{
  uint vectorsize = values.size();

  doubles input(vectorsize,0.0);
  inport.read( input );

  doubles output(vectorsize,0.0);

  for (uint i = 0; i < vectorsize; i++)
  {
    if (input[i] == values[i]) {
      output[i] = 1.0 ;
    }
    else
    {
      output[i] = 0.0 ;
    }
  }
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Equals)
