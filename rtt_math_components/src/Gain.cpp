/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gain.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "Gain.hpp"

using namespace RTT;
using namespace MATH;

Gain::Gain(const string& name) : TaskContext(name, PreOperational)
{
  addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
  addProperty( "gain", gain );

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}
Gain::~Gain(){}

bool Gain::configureHook()
{
  return true;
}

bool Gain::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() )
  {
    log(Error)<<"Gain::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"Gain::Outputport not connected!"<<endlog();
  }
  return true;
}

void Gain::updateHook()
{
  // Read the inputports
  doubles input(vectorsize,0.0);

  inport.read( input );

  // Calculate the output:
  doubles output(vectorsize,0.0);
  for ( uint i = 0; i < vectorsize; i++ )
  {
    output[i] = gain * input[i];
  }

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Gain)
