#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogOutsPera.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogOutsPera::AnalogOutsPera(const string& name) : TaskContext(name, PreOperational)
{
  addEventPort( "in_ev", rpera_port );
  addPort( "out1", out_port1 );
  addPort( "out2", out_port2 );
  addPort( "out3", out_port3 );
}
AnalogOutsPera::~AnalogOutsPera(){}

bool AnalogOutsPera::configureHook()
{
amsg1.values.assign(3,0.0);
amsg2.values.assign(3,0.0);
amsg3.values.assign(3,0.0);
output1.assign(3,0.0);
output2.assign(3,0.0);
output3.assign(3,0.0);
rpera.assign(9,0.0);

return true;
}

bool AnalogOutsPera::startHook()
{
  return true;
}

void AnalogOutsPera::updateHook()
{
  if ( NewData == rpera_port.read(rpera))   // this if statement loads the new data from rpera_port into vector rpera and splits them into three output vectors for each slave
	{
	for ( uint i = 0; i < 3; i++ ) // Slave.1002 (and Slave.1005)
		output1[i] = rpera[i];
    for ( uint i = 3; i < 6; i++ ) // Slave.1003 (and Slave.1006)
		output2[i-3] = rpera[i];
    for ( uint i = 6; i < 9; i++ ) // Slave.1004 (and Slave.1007)
		output3[i-6] = rpera[i];
	}

  for ( uint i = 0; i < 3; i++ )
  	{
    amsg1.values[i] = output1[i];
    amsg2.values[i] = output2[i];
    amsg3.values[i] = output3[i];
    }
    out_port1.write(amsg1);
    out_port2.write(amsg2);
    out_port3.write(amsg3);
}

ORO_CREATE_COMPONENT(SOEM::AnalogOutsPera)
