#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogInsPera2.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogInsPera2::AnalogInsPera2(const string& name) : TaskContext(name, PreOperational)
{
	// Creating Properties	
	addProperty( "number_boards", N ).doc("Number of I/O boards that is connected");
}

AnalogInsPera2::~AnalogInsPera2(){}

bool AnalogInsPera2::configureHook()
{ 
	log(Error) << "AnalogInsPera2: DEPRECATED COMPONENT. Use AnalogInsGeneric" << endlog();
	
	if (N > maxN)
	{
		log(Error)<<"You're trying to connect "<<N<<" I/O while a maximum of "<<maxN<<" is hardcoded, please increase maxN. Apologies"<<endlog();
		return false;
	}
	else if (N == 0)
	{
		log(Warning)<<"Number of boards not specified"<<endlog();
		return false;
	}
	log(Info)<<"Creating ports for "<<N<<" I/O boards."<<endlog();
	
	// Creating inports
	for ( uint i = 0; i < N; i++ )
	{
		// Force inputs
		string name_inport = "in_for"+to_string(i+1);
		if (i != 0) addPort( name_inport, inport_for[i] );
		// One port needs to be an eventport
		else if (i == 0) addEventPort( name_inport, inport_for[i] );
		
		// Position inputs
		name_inport = "in_pos"+to_string(i+1);
		addPort( name_inport, inport_pos[i] );
	}
	 
	// Creating outports
	addPort("out_for", outport_for);
    addPort("out_pos", outport_pos);
  
	// Initialize member variables
	output_for.assign(3*N, 0.0);
	output_pos.assign(3*N, 0.0);

	return true;
  
}

bool AnalogInsPera2::startHook()
{
	for (unsigned int i = 0; i < N; i++) {
		if ( !inport_for[i].connected() || !inport_pos[i].connected() ) {
			log(Error) << "ANALOGINSPERA: One of the inputs is not connected, cannot start component" << endlog();
			return false; 
		}
	}
	return true;
}

void AnalogInsPera2::updateHook()
{
	// Loop over inputports
    for (unsigned int i = 0; i < N; i++) {
		inport_for[i].read(amsgf);
		inport_pos[i].read(amsgp);
		
		// Loop over values and copy these into output message
		for (unsigned int j = 0; j < 3; j++) {
			output_for[3*i+j] = amsgf.values[j];
			output_pos[3*i+j] = amsgp.values[j];
		}
	}

    outport_for.write(output_for);
    outport_pos.write(output_pos);

}

ORO_CREATE_COMPONENT(SOEM::AnalogInsPera2)
