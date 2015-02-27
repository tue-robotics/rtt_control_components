/** MotorCharacteristics.cpp
 *
 * @class MotorCharacteristics
 *
 * \author Max Baeten
 * \date September, 2014
 * \version 1.0
 * To Do: Fix output of derivative, make derivative such that it can be called twice
 * or make derivative as port
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Current2VoltageFFW.hpp"

using namespace std;
using namespace RTT;
using namespace FILTERS;

Current2VoltageFFW::Current2VoltageFFW(const string& name) : 
	    TaskContext(name, PreOperational),
	    N(0), Ts(0.0)
{
	// ports
	addPort( 		"in_position", 	inport_position )	.doc("Port for position input");
	addEventPort(	"in_current", 	inport_current )	.doc("Port for position input");
	addPort( 		"out", 			outport )			.doc("Outport for position input");
	
	// Properties
	addProperty( 	"TS", 			Ts )				.doc("A double value that specifies the sampling time. Should match the sampling time of the component that triggers it (EventPort)");
	addProperty( 	"vector_size", 	N )					.doc("An unsigned integer that specifies the vector size of the in and outputs.");
	addProperty( 	"KE", 			Ke )				.doc("A vector containing motor voltage constants");
	addProperty( 	"GR", 			GR )				.doc("A vector containing gear ratios");
	addProperty( 	"RA", 			Ra )				.doc("A vector containing terminal resistances");
	addProperty( 	"Volt2PWM", 	Volt2PWM )			.doc("A vector containing gains to multiply voltage with for example for PWM controlled motors");
}

Current2VoltageFFW::~Current2VoltageFFW(){}

bool Current2VoltageFFW::configureHook()
{
	// Init for calculatederivative
	previous_input.resize(N);
	previous_output.resize(N);
	a.resize(2);
	b.resize(2);
	double N = 100.0;
	double x = N*Ts+1;
	a[0] =   1;
	a[1] =  -1 / x;
	b[0] =  N / x;
	b[1] = -N / x;

	return true;
}

bool Current2VoltageFFW::startHook()
{
	// Check Ports:
	if ( !inport_current.connected() || !inport_position.connected()) {
		log(Error)<<"Current2VoltageFFW: One of the input ports not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Current2VoltageFFW: Output port not connected!"<<endlog();
	}

	// Check Properties
	if (N < 1 || Ts <= 0) {
		log(Error)<<"Current2VoltageFFW: N or Ts parameters not valid!"<<endlog();
		return false;
	}

	if (Ke.size() != N || GR.size() != N || Ra.size() != N ) {
		log(Error)<<"Current2VoltageFFW: MotorVoltageConstant Ke, GearRatio GR or TerminalResistance Ra parameters wrongly sized!"<<endlog();
		return false;
	}
	for (uint i = 0; i < N; i++) {
		if (Ke[i] < 0.0 || GR[i] < 0.0 || Ra[i] < 0.0 ) {
			log(Error)<<"Current2VoltageFFW: MotorVoltageConstant Ke, GearRatio GR or TerminalResistance Ra parameters erroneus parameters!"<<endlog();
			return false;
		}
	}

	for (uint i = 0; i < N; i++) {
		previous_input[i] = 0.0;
		previous_output[i] = 0.0;
	}

	return true;
}

void Current2VoltageFFW::updateHook()
{
	// Read position and torque input
	doubles position(N,0.0);
	inport_position.read(position);
	doubles current(N,0.0);
	inport_current.read(current);

	// Differentiate position
	doubles velocity(N,0.0);
	velocity = calculatederivative(position);

	// Calculate Voltage
	doubles voltage(N,0.0);
	for (uint i = 0; i < N; i++) {
		voltage[i] = (Ke[i]*velocity[i]/GR[i] + Ra[i]*current[i])  *  Volt2PWM[i];
	}

	// Write Output
	outport.write(voltage);
}

doubles Current2VoltageFFW::calculatederivative(doubles diff_in)
{
	// Determine Dt
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
	dt = (new_time - old_time);
	old_time = new_time;

	//  Determine Derivative
	doubles diff_out(N,0.0);
	if (dt > Ts*0.9 && dt < Ts*1.1) {
		for (uint i = 0; i < N; i++) {
			diff_out[i]  = b[0] * diff_in[i];
			diff_out[i] += b[1] * previous_input[i];
			diff_out[i] -= a[1] * previous_output[i];
			}
		} else {
		diff_out = previous_output;
	}
	previous_input  = diff_in;
	previous_output = diff_out;

	return diff_out;
}

ORO_CREATE_COMPONENT(FILTERS::Current2VoltageFFW)
