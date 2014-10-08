/** DoublesToROS.cpp
 *
 * @class DoublesToROS
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DoublesToROS.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

DoublesToROS::DoublesToROS(const string& name) :
	TaskContext(name, PreOperational)
{
	// Adding ports
	Ndouble = 1;
	addPort( "in", doubleinport );
	addProperty( "NumberOfDoublesInVector", Ndouble );
}

DoublesToROS::~DoublesToROS(){}

bool DoublesToROS::configureHook()
{
	Logger::In in("DoublesToROS::Configure");	
	
	for ( uint i = 0; i < Ndouble; i++ ) { 
		string name_outport = "out"+to_string(i+1);
		addPort( name_outport, doubleoutports[i] );
	}
	return true;
}

bool DoublesToROS::startHook()
{
	Logger::In in("DoublesToROS::Start");	
	
	return true;
}

void DoublesToROS::updateHook()
{
	Logger::In in("DoublesToROS::Update");	
	
	doubles values;
	if ( doubleinport.read( values ) == NewData ) {
		for ( uint i = 0; i < Ndouble; i++ ) {
			std_msgs::Float32 msg;
			msg.data = values[i];
			doubleoutports[i].write( msg );
		}
	}
}

ORO_CREATE_COMPONENT(ROS::DoublesToROS)
