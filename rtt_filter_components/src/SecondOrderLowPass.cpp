/** SecondOrderLowPass.cpp
*
* @class SecondOrderLowPass
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SecondOrderLowPass.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

SecondOrderLowPass::SecondOrderLowPass(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0), Ts(0.0)
{

  addProperty( "vector_size", vector_size );
  addProperty( "sampling_time", Ts );	
  addProperty( "pole_frequency", fp );
  addProperty( "pole_damping", dp );

  // Adding ports
  addEventPort( "in", inport );
  addPort( "out", outport );
}

SecondOrderLowPass::~SecondOrderLowPass(){}

bool SecondOrderLowPass::configureHook()
{
  a0.resize(vector_size);
  a1.resize(vector_size);
  a2.resize(vector_size);
  b0.resize(vector_size);
  b1.resize(vector_size);
  b2.resize(vector_size);
  previous_input.resize(vector_size);
  previous_output.resize(vector_size);
  second_previous_input.resize(vector_size);
  second_previous_output.resize(vector_size);
  
  for (uint i = 0; i < vector_size; i++) {

	double wp = 2*PI*fp[i]+eps;
	double alpha = wp/(tan(wp*Ts/2));

	double x  = 2*PI*fp[i];
	double x1 = pow(alpha/x,2)+2*dp[i]*alpha/x+1;
	double x2 = 2-2*pow(alpha/x,2);
	double x3 = pow(alpha/x,2)-2*dp[i]*alpha/x+1;

	// Numerator and denominator of the filter
	a0[i] = 1;
	a1[i] = x2 / x1;
	a2[i] = x3 / x1;
	b0[i] = 1  / x1;
	b1[i] = 2  / x1;
	b2[i] = 1  / x1;
  }

  
  return true;
}

bool SecondOrderLowPass::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"SecondOrderLowPass::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  
  if ( !outport.connected() ) {
    log(Warning)<<"SecondOrderLowPass::Outputport not connected!"<<endlog();
  }
   
  if (vector_size < 1 || Ts <= 0.0) {
    log(Error)<<"SecondOrderLowPass parameters not valid!"<<endlog();
    return false;
  }
  
  for (uint i = 0; i < vector_size; i++) {
	  if (fp[i] <= 0.0 || dp[i] <= 0.0){
		    log(Error)<<"SecondOrderLowPass parameters not valid!"<<endlog();
			return false;
		}
  }

  for (uint i = 0; i < vector_size; i++) {
	previous_input[i]  = 0.0;
	previous_output[i] = 0.0;
	second_previous_input[i]  = 0.0;
	second_previous_output[i] = 0.0;	
  }
  
  return true;
}

void SecondOrderLowPass::updateHook()
{
 	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);

	inport.read( input );

	for (uint i = 0; i < vector_size; i++) {
		output[i]  = b0[i] * input[i];
		output[i] += b1[i] * previous_input[i];
		output[i] += b2[i] * second_previous_input[i];
		output[i] -= a1[i] * previous_output[i];    
		output[i] -= a2[i] * second_previous_output[i]; 
	}

    second_previous_output = previous_output;
	second_previous_input = previous_input;
	previous_output = output;
	previous_input = input;
 
	// Write the outputs
	outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::SecondOrderLowPass)
