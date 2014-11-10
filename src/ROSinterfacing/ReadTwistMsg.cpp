#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReadTwistMsg.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

ReadTwistMsg::ReadTwistMsg(const string& name) : TaskContext(name, PreOperational)
{
	// Adding Properties:
	addProperty( "max_start_vel", max_start_vel ).doc("Safety, max input velocity at startup");
	addProperty( "max_vel", max_vel ).doc("Maximum velocities");
	addProperty( "max_acc", max_acc ).doc("Maximum acceleration");
	addProperty( "max_interval", max_interval ).doc("Safety, max interval between input messages");

	// Creating ports:
	addEventPort( "cmd_vel", inport );
	addPort( "out", outport_vel );
	addPort("out_acc", outport_acc);
}
ReadTwistMsg::~ReadTwistMsg(){}

bool ReadTwistMsg::configureHook()
{
	
	// Declare names for logging purpose:
	names.push_back("x");
	names.push_back("y");
	names.push_back("phi");


	if ( max_acc.size() != 3 || max_vel.size() !=3 ) {
		log(Error)<<"ReadTwistMsg::max_acc or max_vel not well specified!"<<endlog();
		return false;
	}

	vel_saturated.assign(3,false);
	vel_saturated_print = true;

	return true;
}

bool ReadTwistMsg::startHook()
{
	
	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"ReadTwistMsg::Inputport not connected!"<<endlog();
		return false;
	}
	if ( !outport_vel.connected() ) {
		log(Warning)<<"ReadTwistMsg::Outputport not connected!"<<endlog();
	}

	//Check if values are zero (small) at start
	geometry_msgs::Twist cmd_veldata;
	inport.read(cmd_veldata);
	
	if ( cmd_veldata.linear.x > max_start_vel ) {
		log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.linear.x<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
		return false;
	}
	if ( cmd_veldata.linear.y > max_start_vel ) {
		log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.linear.y<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
		return false;
	}
	if ( cmd_veldata.angular.z > max_start_vel ) {
		log(Error)<<"Cannot start if the reference velocity has a value of "<<cmd_veldata.angular.z<<". A maximum of "<<max_start_vel<<" is allowed"<<endlog();
		return false;
	}
	
	aquisition_time = 0;//os::TimeService::Instance()->getNSecs()*1e-9;     (Hack)
	status = 0;

	for ( uint i = 0; i <= 2; i++ ) {
		previous_references[i] = 0.0;
		ref_vel_prev[i] = 0.0;
	}

	old_time = os::TimeService::Instance()->getNSecs()*1e-9;

	return true;
}


void ReadTwistMsg::updateHook()
{
	
	// Determine timestamp:
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
	double dt = new_time - old_time;
	old_time = new_time;

	// Read the inputport
	geometry_msgs::Twist cmd_veldata;
	doubles references(3,0.0);
	doubles output_vel(3,0.0);
	doubles output_acc(3,0.0);

	// Start fresh data check
	if(NewData == inport.read(cmd_veldata)) {
		receive_interval = new_time - aquisition_time;
		aquisition_time = new_time;
		status = 2;
	}

	double receive_delay = new_time - aquisition_time;
	if ( receive_delay > max_interval ) {
		if ( status == 2 ) {
			log(Warning)<<"Stopped driving because the reference-flow was interrupted."<<endlog();
			status = 1;
		}
		for ( uint i = 0; i <= 2; i++ ) {
			ref_vel[i] = 0.0;
		}
	} else {
		references[0] = cmd_veldata.linear.x;
		references[1] = cmd_veldata.linear.y;
		references[2] = cmd_veldata.angular.z;
		// limit reference velocities
		for ( uint i = 0; i < 3; i++ ) {
			if ( references[i] > max_vel[i] ) {
				references[i] = max_vel[i];
				vel_saturated[i] = true;
			} else if ( references[i] < -max_vel[i] ) {
				references[i] = -max_vel[i];
				vel_saturated[i] = true;
			} else {
				vel_saturated[i] = false;
			}
		}
	}

	// Warning, reference velocity limit exceeded
	if (vel_saturated[0] || vel_saturated[1] || vel_saturated[2] ){
		if (vel_saturated_print) {
			log(Warning) << "Reference velocity exceeds maximum velocity!" << endlog();
			vel_saturated_print = false;
		} else {
			vel_saturated_print = true;
		}
	}

	// Limit output:
	for ( uint i = 0; i <= 2; i++ ) {
		ref_acc[i] = (references[i] - ref_vel_prev[i])/dt;
		ref_acc[i] = max(-max_acc[i], min(max_acc[i], ref_acc[i]));
		ref_vel[i] = ref_vel_prev[i] + ref_acc[i] * dt;
		ref_vel_prev[i] = ref_vel[i];
	}

	// Write data to port
	for ( uint i = 0; i <= 2; i++ ) {
		output_vel[i] = ref_vel[i];
		output_acc[i] = ref_acc[i];
	}

	outport_vel.write( output_vel );
	outport_acc.write( output_acc );
}

void ReadTwistMsg::stopHook()
{
    doubles output_zero(3,0.0);
    outport_vel.write(output_zero);
    outport_acc.write(output_zero);
}

ORO_CREATE_COMPONENT(ROS::ReadTwistMsg)
