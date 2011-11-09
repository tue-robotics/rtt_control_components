#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "AnalogOuts.hpp"

using namespace RTT;
using namespace SOEM;

AnalogOuts::AnalogOuts(const string& name) : TaskContext(name, PreOperational)
{
  max_volt.assign(8,0.0);
  addProperty( "max_volt", max_volt );

  addPort( "Analog_out", Analog_out_port );
  addEventPort( "wheels", wheels_port );
  addEventPort( "spindle", spindle_port );
}
AnalogOuts::~AnalogOuts(){}

bool AnalogOuts::configureHook()
{
  amsg.values.assign(8,0.0);
  amsg.values[0] =  0.008;
  amsg.values[3] = -0.008; // Leakage compensation
  output.assign(8,0.0);
  return true;
}

bool AnalogOuts::startHook()
{
  safe = true;
  return true;
}

void AnalogOuts::updateHook()
{

  if ( NewData == wheels_port.read(wheels))
  {
    for ( uint i = 0; i < 4; i++ )
      output[i] = wheels[i];
  }
  if ( NewData == spindle_port.read(spindle))
  {
    output[4] = spindle;
  }

  for ( uint i = 0; i < 8; i++ )
    if ( output[i] > max_volt[i] )
    {
      safe = false;
      log(Error)<<"Output["<<i<<"] has a value of "<<output[i]<<", stopping"<<endlog();
    }

  if (safe)
    for ( uint i = 0; i < 8; i++ )
      amsg.values[i] = output[i];
  else
    amsg.values.assign(8,0.0);

  Analog_out_port.write(amsg);
}

ORO_CREATE_COMPONENT(SOEM::AnalogOuts)
