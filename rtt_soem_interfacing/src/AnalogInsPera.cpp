#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogInsPera.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogInsPera::AnalogInsPera(const string& name) : TaskContext(name, PreOperational)
{
    addPort("in_for1", inport_for1);
    addPort("in_for2", inport_for2);
    addPort("in_for3", inport_for3);
    addPort("in_pos1", inport_pos1);
    addPort("in_pos2", inport_pos2);
    addPort("in_pos3", inport_pos3);
    addPort("out_for", outport_for);
    addPort("out_pos", outport_pos);
}
AnalogInsPera::~AnalogInsPera(){}

bool AnalogInsPera::configureHook()
{  
	
	log(Error) << "ANALOGINSPERA: DEPRECATED COMPONENT. Use AnalogInsGeneric" << endlog();
  output_for.assign(9, 0.0);
  output_pos.assign(9, 0.0);

  return true;
  
}

bool AnalogInsPera::startHook()
{
	if (!inport_for1.connected() || !inport_for2.connected() || !inport_for3.connected() || !inport_pos1.connected() || !inport_pos2.connected() || !inport_pos3.connected() ) 
	{
		log(Error) << "ANALOGINSPERA: One of the inputs is not connected, cannot start component" << endlog();
		return false; 
	}
	return true;
}

void AnalogInsPera::updateHook()
{
    inport_for1.read(amsgf1);
    inport_for2.read(amsgf2);
    inport_for3.read(amsgf3);
    inport_pos1.read(amsgp1);
    inport_pos2.read(amsgp2);
    inport_pos3.read(amsgp3);

    for ( uint i = 0; i < 3; i++ )
    {
        output_for[i] = amsgf1.values[i];
        output_pos[i] = amsgp1.values[i];
    }
    for ( uint i = 3; i < 6; i++ )
    {
        output_for[i] = amsgf2.values[i-3];
        output_pos[i] = amsgp2.values[i-3];
    }


    for ( uint i = 6; i < 9; i++ )
    {
        output_for[i] = amsgf3.values[i-6];
        output_pos[i] = amsgp3.values[i-6];
    }

    outport_for.write(output_for);
    outport_pos.write(output_pos);

}

ORO_CREATE_COMPONENT(SOEM::AnalogInsPera)
