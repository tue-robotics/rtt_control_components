/**
 * author: Tim Clephas
 * email:  t.t.g.clephas@student.tue.nl
 *
 * filename:             Addition.cpp
 * Last modification:    March 2011
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Addition.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Addition::Addition(const string& name) : TaskContext(name, PreOperational)
{
  addProperty( "vectorsize", vectorsize ).doc("An unsigned integer that specifies the size of the vector");

  // Adding ports
  addPort( "in1", inport1 );
  addEventPort( "in2_event", inport2 );
  addPort( "out", outport );
}
Addition::~Addition(){}

bool Addition::configureHook()
{
  return true;
}

bool Addition::startHook()
{
  // Check validity of Ports:
  if ( !inport1.connected() || !inport2.connected() )
  {
    log(Error)<<"Addition::One or more inputports not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"Addition::Outputport not connected!"<<endlog();
  }
  return true;
}

void Addition::updateHook()
{
  // Read the inputports
  doubles input1(vectorsize,0.0);
  doubles input2(vectorsize,0.0);

  inport1.read( input1 );
  inport2.read( input2 );

  // Calculate the output:
  doubles output(vectorsize,0.0);
  for ( uint i = 0; i < vectorsize; i++ )
  {
    output[i] = input1[i] + input2[i];
  }

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Addition)
