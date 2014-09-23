#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "SaturationCheck.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

SaturationCheck::SaturationCheck(const string& name) : TaskContext(name, PreOperational)
{
	//addPort( "safe_in",safe_inPort).doc("Receives safe = true if no errors are encountered");
	addPort( "safe",enablePort).doc("boolean port to enable amplifier");
	addProperty( "inport_sizes", input_sizes).doc("Vector containing the sizes of each input port");
	addProperty( "motor_saturation", motor_saturation).doc("Vector containing the saturation values");
	addProperty( "max_saturation_time", max_sat_time).doc("Max saturation time");
}

SaturationCheck::~SaturationCheck(){}

bool SaturationCheck::configureHook()
{
	// create the inPorts and count the signals to check
	N_inPorts = input_sizes.size();
	N_signals = 0;
	if ( N_inPorts > 0 ){
		for ( uint i = 0; i < N_inPorts; i++ ){
			string portName = "in"+ to_string(i+1);
			addPort( portName, control_inPort[i] );
			N_signals += input_sizes[i];
		}
	}
	else {
		log(Error)<<"SaturationCheck:: Parameter input_sizes not specified! " << endlog();
		return false;
	}

	// Resizing of inputdata
	inputdata.resize(N_inPorts);
	for ( uint i = 0; i < N_inPorts; i++ ) {
		inputdata[i].resize(input_sizes[i]);
	}

	return true;
}

bool SaturationCheck::startHook()
{ 		
	// check parameters
	if ( sizeof(motor_saturation) != N_signals ){
		log(Error)<<"SaturationCheck:: Parameter motor_saturation wrong size!"<< endlog();
		return false;
	}
	for ( uint i = 0; i < N_signals; i++ ){
		if ( motor_saturation[i] < 0.0 ){
			log(Error)<<"SaturationCheck:: Values in parameter motor_saturation invallid!"<<endlog();
			return false;
		}
	}
	if ( max_sat_time < 0.0 ){
		log(Error)<<"SaturationCheck:: Parameter max_saturation_time invallid!"<<endlog();
		return false;
	}

	// check connections
	if ( !enablePort.connected() ){
		log(Warning)<<"SaturationCheck:: Port enable not connected!"<<endlog();
	}
	for ( uint i = 0; i < N_inPorts; i++ ){
		if ( !control_inPort[i].connected() ){
			log(Warning)<<"SaturationCheck:: Port in"<<i+1<<" not connected!"<<endlog();
		}
	}

	enable = true;
	firstSatInstance.assign(N_signals,true);
	timeReachedSaturation.assign(N_signals,0.0);

	return true;
}

void SaturationCheck::updateHook()
{
	// get time
	long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;

	// get input data
	for ( uint i=0; i<N_inPorts; i++ ){
		if ( NewData == control_inPort[i].read(inputdata[i])) { }
	}

	// check all the motor values on saturation
	uint port_iterator = 0;
	uint start_port = 0;
	for ( uint i=0; i<N_signals; i++){
		// switch to next port if necessary
		if ( i-start_port > input_sizes[port_iterator]-1 ){
			start_port = start_port + input_sizes[port_iterator];
			port_iterator++;
		}
		double data = inputdata[port_iterator][i-start_port];

		// check saturation of motor
		if(firstSatInstance[i] && fabs(data)>=motor_saturation[i]){
			timeReachedSaturation[i]=timeNow;
			firstSatInstance[i]=false;
		}
		else if(fabs(data)<motor_saturation[i]){
			timeReachedSaturation[i]=timeNow;
			firstSatInstance[i]=true;
		}
		if(fabs(timeNow-timeReachedSaturation[i])>=max_sat_time){
			if(enable==true){
				ROS_ERROR_STREAM( "Controller: Motor output "<<i+1<<" satured too long (absolute "<<max_sat_time<<" sec above "<<fabs(motor_saturation[i])<<"). output disabled." );
				enable = false;
			}
		}
	}

	// send enable signal
	enablePort.write(enable);
}

ORO_CREATE_COMPONENT(SUPERVISORY::SaturationCheck)
