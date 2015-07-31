#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DigitalIns.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

DigitalIns::DigitalIns(const string& name) : TaskContext(name, PreOperational)
{
	for ( uint i = 0; i < 8; i++ ) {
		addPort("out"+to_string(i+1), outport[i]);
		flip[i] = false;
		addProperty("flip_out"+to_string(i+1), flip[i]);
	} 
	addPort( "in", inport );
}
DigitalIns::~DigitalIns(){}

bool DigitalIns::configureHook()
{
  return true;
}

bool DigitalIns::startHook()
{
  return true;
}

void DigitalIns::updateHook()
{
    soem_beckhoff_drivers::DigitalMsg dmsg;
	std_msgs::Bool newmsg;
		
	inport.read(dmsg);
	for ( uint i = 0; i < dmsg.values.size(); i++ ) {
		if (flip[i]) {
			newmsg.data = !dmsg.values[i];
		} else {
			newmsg.data = dmsg.values[i];
		}
		outport[i].write(newmsg);
	}
}

ORO_CREATE_COMPONENT(SOEM::DigitalIns)
