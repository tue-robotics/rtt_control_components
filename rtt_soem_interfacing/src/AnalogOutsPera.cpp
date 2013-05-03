#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogOutsPera.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogOutsPera::AnalogOutsPera(const string& name) : TaskContext(name, PreOperational)
{
  max_volt.assign(8,0.0);
  addProperty( "max_volt", max_volt );

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
rpera.assign(8,0.0);
return true;
}

bool AnalogOutsPera::startHook()
{
  safe = true;
  return true;
}

void AnalogOutsPera::updateHook()
{
  if ( NewData == rpera_port.read(rpera))   // this if statement loads the new data from rpera_port into vector rpera and splits them into three output vectors for each slave
  {	output2[0] = rpera[0];
	output2[1] = rpera[1];
	
    output2[0] = rpera[2];
	output2[1] = rpera[3];
	output2[2] = rpera[4];
	
    output3[0] = rpera[5];
    output3[1] = rpera[6];
    output3[2] = rpera[7];
  }

  for ( uint i = 0; i < 8; i++ )
    if ( rpera[i] > max_volt[i] )
    {
      safe = false;
      log(Error)<<"Output["<<i<<"] has a value of "<<rpera[i]<<", stopping" <<endlog();
    }

  if (safe)
      for ( uint i = 0; i < 3; i++ )
      {
        amsg1.values[i] = output1[i];
        amsg2.values[i] = output2[i];
        amsg3.values[i] = output3[i];
      }
  else
    {
      for ( uint i = 0; i < 3; i++ )
      {
        amsg1.values[i] = 0.0;
        amsg2.values[i] = 0.0;
        amsg3.values[i] = 0.0;
      }
    }

    out_port1.write(amsg1);
    out_port2.write(amsg2);
    out_port3.write(amsg3);
}

ORO_CREATE_COMPONENT(SOEM::AnalogOutsPera)
