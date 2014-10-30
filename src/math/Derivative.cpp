/** Derivative.cpp
 *
 * @class Derivative
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Derivative.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Derivative::Derivative(const string& name) : 
	    TaskContext(name, PreOperational),
	    vector_size(0), Ts(0.0)
{
  addProperty( "sampling_time", Ts ).doc("A double value that specifies the sampling time. Should match the sampling time of the component that triggers it (EventPort)");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the vector.");
}

Derivative::~Derivative(){}

bool Derivative::configureHook()
{
  Logger::In in("Derivative::configureHook()");

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );

  previous_input.resize(vector_size);
  previous_output.resize(vector_size);
  a.resize(2);
  b.resize(2);

  // Filter coefficient N
  double N = 100.0;

  double x = N*Ts+1;

  a[0] =   1;
  a[1] =  -1 / x;
  b[0] =  N / x;
  b[1] = -N / x;

  return true;
}

bool Derivative::startHook()
{
  Logger::In in("Derivative::startHook()");

  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }

  if (vector_size < 1 || Ts <= 0) {
    log(Error)<<"Derivative parameters not valid!"<<endlog();
    return false;
  }

  for (uint i = 0; i < vector_size; i++) {
    previous_input[i]  = 0.0;
    previous_output[i]  = 0.0;
  }

  return true;
}

void Derivative::updateHook()
{
  doubles input(vector_size,0.0);
  doubles output(vector_size,0.0);

  // Read the input port
  inport.read( input );

  determineDt();

  if (dt > Ts*0.9 && dt < Ts*1.1) {
    for (uint i = 0; i < vector_size; i++) {
      output[i]  = b[0] * input[i];
      output[i] += b[1] * previous_input[i];
      output[i] -= a[1] * previous_output[i];
    }
  } else {
    output = previous_output;
  }

  previous_input  = input;
  previous_output = output;

  // Write the outputs
  outport.write( output );
}

void Derivative::determineDt()
{
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  dt = (new_time - old_time);
  old_time = new_time;
}

ORO_CREATE_COMPONENT(MATH::Derivative)
