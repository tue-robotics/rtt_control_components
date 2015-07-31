#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DigitalOuts.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

DigitalOuts::DigitalOuts(const string& name) : TaskContext(name, PreOperational),
												n_bits(8)
{
	addProperty( "number_of_bits", n_bits ).doc("The number of bools that are sent to the slave (MAX 8).");
	addPort( "digital_out", digital_out_port );
}
DigitalOuts::~DigitalOuts(){}

bool DigitalOuts::configureHook()
{
	if (n_bits < 1 || n_bits > 8) {
		log(Error) << "DigitalOuts::Wrong number_of_bits specified, (0< n < 9)." << endlog();
		return false;
	}

	for ( uint i = 0; i < n_bits; i++ ) {
		addEventPort("in"+to_string(i+1), inport[i]);
	}
	
	dmsg.values.assign(n_bits,false);
	
	return true;
}

bool DigitalOuts::startHook()
{
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	return true;
}

void DigitalOuts::updateHook()
{
	long double current_time = os::TimeService::Instance()->getNSecs()*1e-9;

	if ( current_time -  start_time > 1.0 ) { //enable hardware after one second delay 
		for (unsigned int i = 0; i < dmsg.values.size(); i++) {
			bool data = false;
			if ( inport[i].read(data) == NewData ) {
				dmsg.values[i] = data;
			}
		}
		digital_out_port.write(dmsg);
	}
}

ORO_CREATE_COMPONENT(SOEM::DigitalOuts)
