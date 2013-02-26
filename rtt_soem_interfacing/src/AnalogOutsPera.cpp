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

  addPort( "out1", out_port1 );
  addPort( "out2", out_port2 );
  addPort( "out3", out_port3 );
  addEventPort( "in_ev", rpera_port );
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
  {
    for ( uint i = 0; i < 2; i++ ) // Slave.1002 (and Slave.1005)
      output1[i] = rpera[i];
    for ( uint i = 2; i < 5; i++ ) // Slave.1003 (and Slave.1006)
      output2[i-2] = rpera[i];
    for ( uint i = 5; i < 8; i++ ) // Slave.1004 (and Slave.1007)
      output3[i-5] = rpera[i];
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
        log(Error)<<"AnalogOutsPera" <<endlog();
      }
    }

    out_port1.write(amsg1);
    out_port2.write(amsg2);
    out_port3.write(amsg3);
}

ORO_CREATE_COMPONENT(SOEM::AnalogOutsPera)
