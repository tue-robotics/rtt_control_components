/** AnyToROS.cpp
 *
 * @class AnyToROS
 *
 * \author Max Baeten
 * \date Nov, 2016
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnyToROS.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

AnyToROS::AnyToROS(const string& name) :
    TaskContext(name, PreOperational)
{
	//! Operations
	// Add Ins
    addOperation("AddDoublesToROS", &AnyToROS::AddDoublesToROS, this, OwnThread)
		.doc("Add a doubles to ROS")
		.arg("N","vector size")
		.arg("portname","port name")
		.arg("ids","To select a certain number of elements");
}

AnyToROS::~AnyToROS(){}

bool AnyToROS::configureHook()
{
    n_ports_D = 0;
        
    return true;
}

bool AnyToROS::startHook()
{
    return true;
}

void AnyToROS::AddDoublesToROS(int N, ints ids, string portname)
{
	log(Warning) << "AnyToROS::AddDoublesToROS: ids.size() = " << ids.size() << "!" << endlog();
	
	// Check
	if(n_ports_D >= maxPorts) {
        log(Error) << "AnyToROS::AddDoublesToROS: Could not add doubles to ROS. Already to many ports: " << n_ports_D << " added. maxPorts: " << maxPorts << "!" << endlog();
	}
	if(ids.size() > N) {
        log(Error) << "AnyToROS::AddDoublesToROS: Could not add doubles to ROS. Since the length of vector ids: " << ids.size() << ", is longer than the total number of joints: " << N << "!" << endlog();
	}
	
	n_ports_D++;
	
	// Init inputs
	input_D[n_ports_D-1].resize(N);
	for(uint j=0; j<N; j++) {
		input_D[n_ports_D-1][j]=0.0;
	}
	
	// Init outputs	
	ids_D[n_ports_D-1].resize(ids.size());	
	for(uint j=0; j<ids.size(); j++) {
		ids_D[n_ports_D-1][j]=ids[j];
	}
	
	// Add ports
	if (n_ports_D==1) {
		addEventPort( "in_"+portname+"Ev", inports_D[n_ports_D-1]);
	} else {
		addPort( "in_"+portname, inports_D[n_ports_D-1]);
	}	
	
	for(uint j=0; j<ids.size(); j++) {	
		
		addPort( "out_"+portname+"_"+to_string(ids[j]), outports_D[n_ports_D-1][j] );
		addPort( "outmsg_"+portname+"_"+to_string(ids[j]), outports_D_msg[n_ports_D-1][j] );
	}

}

void AnyToROS::updateHook()
{	
	static unsigned int input_index;  // Indicate which element from the input vector is desired
	static std_msgs::Float32 output_msg; // Message to send
	
	// Doubles to ROS
	for(uint i=0; i<n_ports_D; i++) {									// i loops over the number of inports
		if ( inports_D[i].read(input_D[i]) == NewData ) {
			for(uint k=0; k<ids_D[i].size(); k++) {						// k loops over the the number of requested output ports
				// Get the index for the input data
				input_index = ids_D[i][k]-1;
				
				// Get the data from the input struct
				double data_point = input_D[i][input_index];
				
				// fill in the output message and write it to the port
				output_msg.data = data_point;
				outports_D_msg[i][k].write(output_msg);
				
				// write to normal port
				outports_D[i][k].write(data_point);
			}
		}
	}
}

ORO_CREATE_COMPONENT(ROS::AnyToROS)
