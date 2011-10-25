/** FirstOrderLowPass.cpp
 *
 * @class FirstOrderLowPass
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "FirstOrderLowPass.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace RTT;
using namespace FILTERS;

FirstOrderLowPass::FirstOrderLowPass(const string& name) : 
	    TaskContext(name, PreOperational),
	    vector_size(0), Ts(0.0)
{
  addProperty( "pole_frequency", fp );
  addProperty( "vector_size", vector_size );
  addProperty( "sampling_time", Ts );	
}

FirstOrderLowPass::~FirstOrderLowPass(){}

bool FirstOrderLowPass::configureHook()
{
  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
  
  /* Guaranteeing Real-Time data flow */
  // create an example data sample of vector_size:
  doubles example(vector_size, 0.0);

  // show it to the port (this is a not real-time operation):
  outport.setDataSample( example );
  /* Guaranteeing Real-Time data flow */

  a[0].resize(vector_size);
  a[1].resize(vector_size);
  b[0].resize(vector_size);
  b[1].resize(vector_size);
  previous_input.resize(vector_size);
  previous_output.resize(vector_size);

  for (uint i = 0; i < vector_size; i++) {

    double wp = 2*PI*fp[i]+eps;
    double alpha = wp/(tan(wp*Ts/2));

    double x1 = alpha/(2*PI*fp[i])+1;
    double x2 = -alpha/(2*PI*fp[i])+1;

    // Numerator and denominator of the filter
    a[0][i] = 1;
    a[1][i] = x2 / x1;
    b[0][i] = 1  / x1;
    b[1][i] = 1  / x1;
  }

  return true;
}

bool FirstOrderLowPass::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"FirstOrderLowPass::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"FirstOrderLowPass::Outputport not connected!"<<endlog();
  }

  if (vector_size < 1 || Ts <= 0.0) {
    log(Error)<<"FirstOrderLowPass parameters not valid!"<<endlog();
    return false;
  }

  for (uint i = 0; i < vector_size; i++) {
    if (fp[i] <= 0.0){
      log(Error)<<"FirstOrderLowPass parameters not valid!"<<endlog();
      return false;
    }
  }

  for (uint i = 0; i < vector_size; i++) {
    previous_input[i]  = 0.0;
    previous_output[i] = 0.0;
  }

  return true;
}

void FirstOrderLowPass::updateHook()
{
  // Read the input port
  doubles input(vector_size,0.0);
  doubles output(vector_size,0.0);

  inport.read( input );

  for (uint i = 0; i < vector_size; i++) {
    output[i]  = b[0][i] * input[i];
    output[i] += b[1][i] * previous_input[i];
    output[i] -= a[1][i] * previous_output[i];
  }

  previous_output = output;
  previous_input = input;

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::FirstOrderLowPass)
