/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Gains.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "Gains.hpp"

using namespace RTT;
using namespace MATH;

Gains::Gains(const string& name) : TaskContext(name, PreOperational)
{
  addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");
  addProperty( "gain", gain ).doc("An array containing the gain values");

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}
Gains::~Gains(){}

bool Gains::configureHook()
{
  return true;
}

bool Gains::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() )
  {
    log(Error)<<"Gains::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() )
  {
    log(Warning)<<"Gains::Outputport not connected!"<<endlog();
  }
  if ( vectorsize != gain.size() )
  {
    log(Error)<<"Gains::vectorsizes is "<<vectorsize<<" while gain array is "<<gain.size()<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  return true;
}

void Gains::updateHook()
{
  // Read the inputports
  doubles input(vectorsize,0.0);

  inport.read( input );

  // Calculate the output:
  doubles output(vectorsize,0.0);
  for ( uint i = 0; i < vectorsize; i++ )
  {
    output[i] = gain[i] * input[i];
  }

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Gains)
