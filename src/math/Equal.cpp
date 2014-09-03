/** Equal.cpp
 *
 * @class Equal
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Equal.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Equal::Equal(const string& name) :
	    TaskContext(name, PreOperational)
{
  // Adding property
  addProperty( "values", values );
  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}

Equal::~Equal(){}

bool Equal::configureHook()
{
  return true;
}

bool Equal::startHook()
{
  Logger::In in("Equal::startHook()");

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

void Equal::updateHook()
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

ORO_CREATE_COMPONENT(MATH::Equal)
