/** PID.cpp
 *
 * @class PID
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "PID.hpp"

using namespace std;
using namespace RTT;
using namespace FILTERS;

PID::PID(const string& name) : 
	    TaskContext(name, PreOperational),
			Ts(0.0), vector_size(0)
{

  addProperty( "proportional_coefficient", kp ).doc("Array of proportional coefficients of PID.");
  addProperty( "derivative_coefficient", kv ).doc("Array of derivative coefficients of PID.");
  addProperty( "integral_coefficient", ki ).doc("Array of integral coefficients of PID.");
  addProperty( "anti_windup_coefficient", kaw ).doc("Array of anti-windup coefficients of PID.");
  addProperty( "integrator_initial_value", init ).doc("Array of integrator initial values.");
  addProperty( "limit", limit ).doc("Array of saturation limits.");
  addProperty( "sampling_time", Ts).doc("Sampling time.");
  addProperty( "vector_size", vector_size ).doc("Size of the input vector.");

}

PID::~PID(){}

bool PID::configureHook()
{
  Logger::In in("PID::configureHook()");

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
  a[2].resize(vector_size);
  b[0].resize(vector_size);
  b[1].resize(vector_size);
  b[2].resize(vector_size);
  previous_input.resize(vector_size);
  previous_output.resize(vector_size);
  second_previous_input.resize(vector_size);
  second_previous_output.resize(vector_size);
  windup.resize(vector_size);
  anti_windup.resize(vector_size);

  // Filter coefficient N
  double N = 100.0;

  for (uint i = 0; i < vector_size; i++) {
    double x1 = N*Ts+1;
    double x2 = -1.0-x1;
    double x3 = 1;
    double x4 = kv[i]*N+kp[i]*x1;
    double x5 = -2*kv[i]*N-2*kp[i]-kp[i]*N*Ts+ki[i]*Ts+ki[i]*Ts*Ts*N;
    double x6 = kv[i]*N+kp[i]-ki[i]*Ts;

    // Numerator and denominator of the filter
    a[0][i] = 1;
    a[1][i] = x2 / x1;
    a[2][i] = x3 / x1;
    b[0][i] = x4 / x1;
    b[1][i] = x5 / x1;
    b[2][i] = x6 / x1;
  }  

  return true;
}

bool PID::startHook()
{
  Logger::In in("PID::startHook()");

  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  if (Ts <= 0 || vector_size < 1 ) {
    log(Error)<<"PID parameters not valid!"<<endlog();
    return false;
  }

  for (uint i = 0; i < vector_size; i++) {
    if (limit[i] < 0.0) {
      log(Error)<<"Limit can't be negative!"<<endlog();
      return false;
    }
  }

  for (uint i = 0; i < vector_size; i++) {
    previous_input[i]  = 0.0;
    previous_output[i] = 0.0;
    second_previous_input[i]  = 0.0;
    second_previous_output[i] = 0.0;
    windup[i] = 0.0;
    anti_windup[i] = 0.0;
  }

  old_time = os::TimeService::Instance()->getNSecs()*1e-9;

  return true;
}

void PID::updateHook()
{
	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);
	doubles output1(vector_size,0.0);
	doubles output2(vector_size,0.0);

	// Read the input port
	inport.read( input );

	double dt = determineDt();
	
	//log(Warning)<<"dt = "<<dt<<endlog();
	
	for (uint i = 0; i < vector_size; i++) {
		output1[i] += b[0][i] * input[i];
		output1[i] += b[1][i] * previous_input[i];
		output1[i] += b[2][i] * second_previous_input[i];
		output1[i] -= a[1][i] * previous_output[i];    
		output1[i] -= a[2][i] * second_previous_output[i]; 
	}

	second_previous_output = previous_output;
	second_previous_input = previous_input;
	previous_output = output1;
	previous_input = input;


	// Integrator initial value
	for (uint i = 0; i < vector_size; i++) {
		output1[i] += init[i];
	}	
	
	for (uint i = 0; i < vector_size; i++) {
		// Value that compensates the integrator action = anti-windup action
		anti_windup[i] += kaw[i]*windup[i]*dt;
		
		// Value of the output before saturation block
		output2[i] = output1[i] + anti_windup[i];
		
		// Final value of the output
		output[i] = output2[i];
		if (limit[i] > 0.0) {
			if (output2[i] > limit[i]) {
				output[i] = limit[i];
			}
			if (output2[i] < -limit[i]) {
				output[i] = -limit[i];
			}
		}

		// Windup value
		windup[i] = output[i] - output2[i];
	  }

	// Write the outputs
	outport.write( output );
}

double PID::determineDt()
{
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  //log(Debug)<<"new_time"<<new_time<<endlog();
  double dt = (double)(new_time - old_time);
  old_time = new_time;
  return dt;
}

ORO_CREATE_COMPONENT(FILTERS::PID)
