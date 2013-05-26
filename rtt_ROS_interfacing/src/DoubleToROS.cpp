/** DoubleToROS.cpp
 *
 * @class DoubleToROS
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DoubleToROS.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

DoubleToROS::DoubleToROS(const string& name) :
	TaskContext(name, PreOperational)
{
	// Adding ports
	Ndouble = 1;
	addProperty( "NumberOfDoublePorts", Ndouble );
}

DoubleToROS::~DoubleToROS(){}

bool DoubleToROS::configureHook()
{
	for ( uint i = 0; i < Ndouble; i++ )
	{
		string name_inport = "double_in"+to_string(i+1);
		string name_outport = "double_out"+to_string(i+1);
		addPort( name_inport, doubleinports[i] );
		addPort( name_outport, doubleoutports[i] );
	}
	return true;
}

bool DoubleToROS::startHook()
{
	return true;
}

void DoubleToROS::updateHook()
{
	for ( uint i = 0; i < Ndouble; i++ )
	{
		double value;
		if ( doubleinports[i].read( value ) == NewData )
		{
			std_msgs::Float64 msg;
			msg.data = value;
			doubleoutports[i].write( msg );
		}
	}
}

ORO_CREATE_COMPONENT(ROS::DoubleToROS)
