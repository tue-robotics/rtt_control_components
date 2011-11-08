/** LeadLags.cpp
*
* @class LeadLags
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "LeadLags.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace RTT;
using namespace FILTERS;

LeadLags::LeadLags(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0), Ts(0.0)
{

  addProperty( "vector_size", vector_size );
  addProperty( "sampling_time", Ts );
  addProperty( "zero_frequency", fz );
  addProperty( "pole_frequency", fp );

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}

LeadLags::~LeadLags(){}

bool LeadLags::configureHook()
{
  a0.resize(vector_size);
  a1.resize(vector_size);
  b0.resize(vector_size);
  b1.resize(vector_size);
  previous_input.resize(vector_size);
  previous_output.resize(vector_size);
  
  for (uint i = 0; i < vector_size; i++) {

	double wp = 2*PI*sqrt(fz[i]*fp[i])+eps;
	double alpha = wp/(tan(wp*Ts/2)); 

	double x = 2*PI*fz[i]*fp[i];
	double x1 = alpha*fz[i] + x;
	double x2 = -alpha*fz[i] + x;
	double x3 = alpha*fp[i] + x;
	double x4 = -alpha*fp[i] + x;

	// Numerator and denominator of the filter
	a0[i] = 1;
	a1[i] = x2 / x1;
	b0[i] = x3 / x1;
	b1[i] = x4 / x1;
  }

  return true;
}

bool LeadLags::startHook()
{
  // Check validity of ports:
  if ( !inport.connected() ) {
    log(Error)<<"LeadLags::Input port not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !outport.connected() ) {
    log(Warning)<<"LeadLags::Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1 || Ts <= 0.0) {
    log(Error)<<"LeadLags parameters not valid!"<<endlog();
    return false;
  }
  
  for (uint i = 0; i < vector_size; i++) {
	  if (fz[i] <= 0.0 || fp[i] <= 0.0){
		    log(Error)<<"LeadLags parameters not valid!"<<endlog();
			return false;
		}
  }

  for (uint i = 0; i < vector_size; i++) {
	previous_input[i]  = 0.0;
	previous_output[i] = 0.0;
  }

  return true;
}

void LeadLags::updateHook()
{
	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

	inport.read( input );

	for (uint i = 0; i < vector_size; i++) {
		output[i]  = b0[i] * input[i];
		output[i] += b1[i] * previous_input[i];
		output[i] -= a1[i] * previous_output[i];    
	}

	previous_input  = input;
	previous_output = output;

	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::LeadLags)
