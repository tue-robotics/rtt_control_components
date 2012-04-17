/** RosDiagnostics.cpp
 *
 * @class RosDiagnostics
 *
 * \author Tim Clephas
 * \date Istanbul, 2011
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "RosDiagnostics.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

RosDiagnostics::RosDiagnostics(const string& name) :
	                    		    								TaskContext(name, PreOperational)
{
	// Adding ports
	addPort( "diagnostics", diagnosticsport );
	Nvec = 1;
	addProperty( "NumberOfVectorPorts", Nvec );
	Nbool = 0;
	addProperty( "NumberOfBoolPorts", Nbool );
	statusname = "RosDiagnostics";
	addProperty( "statusname", statusname );
}

RosDiagnostics::~RosDiagnostics(){}

bool RosDiagnostics::configureHook()
{
	for ( uint i = 0; i < Nvec; i++ )
	{
		string name_inport = "vec"+to_string(i+1);
		log(Info)<<"Trying to create port "<<name_inport<<endlog();
		addPort( name_inport, vectorports[i] );
		string name_prop = "vecname"+to_string(i+1);
		addProperty( name_prop, vectornames[i] );
	}
	for ( uint i = 0; i < Nbool; i++ )
	{
		string name_inport = "bool"+to_string(i+1);
		log(Info)<<"Trying to create port "<<name_inport<<endlog();
		addPort( name_inport, boolports[i] );
		string name_prop = "boolname"+to_string(i+1);
		addProperty( name_prop, boolnames[i] );
	}
	return true;
}

bool RosDiagnostics::startHook()
{
	return true;
}

void RosDiagnostics::updateHook()
{

	vector<diagnostic_msgs::DiagnosticStatus> statuses;

	diagnostic_updater::DiagnosticStatusWrapper status;
	status.name = statusname;
	for ( uint i = 0; i < Nvec; i++ )
	{
		doubles values;
		vectorports[i].read( values );
		for ( uint j = 0; j < values.size(); j++ )
		{
			status.add(vectornames[i],values[j]);
		}
	}
	for ( uint i = 0; i < Nbool; i++ )
	{
		bool value;
		boolports[i].read( value );
		status.add(boolnames[i],value);
	}

	statuses.push_back(status);


	diagnostic_msgs::DiagnosticArray msg;

	msg.status=statuses;
	msg.header.stamp = ros::Time::now();

	diagnosticsport.write( msg );

}

ORO_CREATE_COMPONENT(ROS::RosDiagnostics)
