/** SkewedNotch.cpp
*
* @class SkewedNotch
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "SkewedNotch.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

SkewedNotch::SkewedNotch(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0),Ts(0.0)
{

  addProperty( "zero_frequency", fz );
  addProperty( "zero_damping", dz );
  addProperty( "pole_frequency", fp );
  addProperty( "pole_damping", dp );
  addProperty( "vector_size", vector_size );
  addProperty( "sampling_time", Ts );	

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}

SkewedNotch::~SkewedNotch(){}

bool SkewedNotch::configureHook()
{
  Logger::In in("SkewedNotch::configureHook()");

  /* Guaranteeing Real-Time data flow */
  // create an example data sample of vector_size:
  doubles example(vector_size, 0.0);

  // show it to the port (this is a not real-time operation):
  outport.setDataSample( example );
  /* Guaranteeing Real-Time data flow */

  a[0].resize(vector_size);
  a[1].resize(vector_size);
  a[2].resize(vector_size);
  b[0].resize(vector_size);
  b[1].resize(vector_size);
  b[2].resize(vector_size);
  previous_input.resize(vector_size);
  previous_output.resize(vector_size);
  second_previous_input.resize(vector_size);
  second_previous_output.resize(vector_size);
  
  for (uint i = 0; i < vector_size; i++) {
	  // Discrete time parameters (Tustin with prewarping)
	  double wp = 2*PI*fz[i]+eps;	//or wp = 2*PI*fp+eps
	  double alpha = wp/(tan(wp*Ts/2));
	  
	  double x1 = pow(alpha/(2*PI*fz[i]),2)+2*dz[i]*alpha/(2*PI*fz[i])+1;
	  double x2 = pow(alpha/(2*PI*fp[i]),2)+2*dp[i]*alpha/(2*PI*fp[i])+1;
	  double x3 = 2-2*pow(alpha/(2*PI*fz[i]),2);
	  double x4 = 2-2*pow(alpha/(2*PI*fp[i]),2);
	  double x5 = pow(alpha/(2*PI*fz[i]),2)-2*dz[i]*alpha/(2*PI*fz[i])+1;
	  double x6 = pow(alpha/(2*PI*fp[i]),2)-2*dp[i]*alpha/(2*PI*fp[i])+1;
	  
	  // Numerator and denominator of the filter
	  a[0][i] = 1;
	  a[1][i] = x4 / x2;
	  a[2][i] = x6 / x2;
	  b[0][i] = x1 / x2;
	  b[1][i] = x3 / x2;
	  b[2][i] = x5 / x2;
  }
 
  return true;
}

bool SkewedNotch::startHook()
{
  Logger::In in("SkewedNotch::startHook()");
  
  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  if (vector_size < 1 || Ts <= 0.0) {
    log(Error)<<"FirstOrderLowPasses parameters not valid!"<<endlog();
    return false;
  }

  for (uint i = 0; i < vector_size; i++) {
	if (fz[i] <= 0.0 || dz[i] <= 0.0 || fp[i] <= 0.0 || dp[i] <= 0.0) {
		log(Error)<<"SkewedNotch parameters not valid!"<<endlog();
		return false;
	}
  }

  for (uint i = 0; i < vector_size; i++) {
    previous_input[i]  = 0.0;
    previous_output[i] = 0.0;
	second_previous_output[i] = 0.0;
	second_previous_input[i] = 0.0;
  }

  return true;
}

void SkewedNotch::updateHook()
{
	Logger::In in("SkewedNotch::updateHook()");

	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

	inport.read( input );

	//double dt = determineDt();

	for (uint i = 0; i < vector_size; i++) {
	output[i]  = b[0][i] * input[i];
	output[i] += b[1][i] * previous_input[i];
	output[i] += b[2][i] * second_previous_input[i];
	output[i] -= a[1][i] * previous_output[i];    
	output[i] -= a[2][i] * second_previous_output[i]; 
	}

	second_previous_output = previous_output;
	second_previous_input  = previous_input;
	previous_output = output;
	previous_input  = input;

	// Write the outputs
	outport.write( output );
}

double SkewedNotch::determineDt()
{
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  log(Debug)<<"new_time"<<new_time<<endlog();
  double dt = (double)(new_time - old_time);
  old_time = new_time;
  return dt;
}

ORO_CREATE_COMPONENT(FILTERS::SkewedNotch)
