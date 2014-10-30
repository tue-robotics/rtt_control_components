#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogIns.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogIns::AnalogIns(const string& name) : TaskContext(name, PreOperational)
{
	for ( uint i = 0; i < 2; i++ ) {
		addPort("out"+to_string(i+1), outport[i]);
	} 
	addPort( "in", inport );
}
AnalogIns::~AnalogIns(){}

bool AnalogIns::configureHook()
{
  return true;
}

bool AnalogIns::startHook()
{
  return true;
}

void AnalogIns::updateHook()
{
    soem_beckhoff_drivers::AnalogMsg amsg;
	std_msgs::Float32 newmsg;
		
	inport.read(amsg);
	for ( uint i = 0; i < amsg.values.size(); i++ ) {
		newmsg.data = amsg.values[i];
		outport[i].write(newmsg);
	}
}

ORO_CREATE_COMPONENT(SOEM::AnalogIns)
