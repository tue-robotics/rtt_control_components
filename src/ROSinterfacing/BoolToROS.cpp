/** BoolToROS.cpp
 *
 * @class BoolToROS
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "BoolToROS.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

BoolToROS::BoolToROS(const string& name) :
	TaskContext(name, PreOperational)
{
	// Adding ports
	Nbool = 1;
	addProperty( "NumberOfBoolPorts", Nbool );
}

BoolToROS::~BoolToROS(){}

bool BoolToROS::configureHook()
{
	Logger::In in("BoolToROS::Configure");		
	
	for ( uint i = 0; i < Nbool; i++ ) {
		string name_inport = "bool_in"+to_string(i+1);
		string name_outport = "bool_out"+to_string(i+1);
		log(Info)<<"Trying to create port "<<name_inport<<endlog();
		addPort( name_inport, boolinports[i] );
		addPort( name_outport, booloutports[i] );
	}
	return true;
}

bool BoolToROS::startHook()
{
	Logger::In in("BoolToROS::Start");	
	
	return true;
}

void BoolToROS::updateHook()
{
	Logger::In in("BoolToROS::Update");		
	
	for ( uint i = 0; i < Nbool; i++ ) {
		bool value;
		if ( boolinports[i].read( value ) == NewData ) {
			std_msgs::Bool msg;
			msg.data = value;
			booloutports[i].write( msg );
		}
	}
}

ORO_CREATE_COMPONENT(ROS::BoolToROS)
