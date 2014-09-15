#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "DigitalOuts.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

DigitalOuts::DigitalOuts(const string& name) : TaskContext(name, PreOperational)
{
  addPort( "digital_out", digital_out_port );
  //addEventPort( "amplifiers", amplifiers_port );
  //addEventPort( "tuelights", tuelights_port );
  //addEventPort( "spindlebrake", spindlebrake_port );
  //addEventPort( "red", red_port );
  //addEventPort( "green", green_port );
  //addEventPort( "blue", blue_port );
  for ( uint i = 0; i < 8; i++ ) 
	{
		addEventPort("in"+to_string(i+1), inport[i]);
	} 
}
DigitalOuts::~DigitalOuts(){}

bool DigitalOuts::configureHook()
{
  dmsg.values.assign(8,false);
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

  if ( current_time -  start_time > 1.0 ) //enable hardware after one second delay 
  {
 /*
	if ( NewData == amplifiers_port.read(amplifiers))
	{
		dmsg.values[0] = amplifiers;
		if ( NewData == tuelights_port.read(tuelights))
			dmsg.values[1] = tuelights;
		if ( NewData == spindlebrake_port.read(spindlebrake))
			dmsg.values[2] = spindlebrake;
	}
	
		bool red;
		bool green;
		bool blue;
 
		if ( NewData == red_port.read(red))
			dmsg.values[5] = red;
		if ( NewData == green_port.read(green))
			dmsg.values[6] = green;
		if ( NewData == blue_port.read(blue))
			dmsg.values[7] = blue;    

		digital_out_port.write(dmsg);
		*/
	for (unsigned int i = 0; i < dmsg.values.size(); i++)
	{
		bool data = false;
		if ( inport[i].read(data) == NewData )
		{
			dmsg.values[i] = data;
		}
	}
	digital_out_port.write(dmsg);
  }
}

ORO_CREATE_COMPONENT(SOEM::DigitalOuts)