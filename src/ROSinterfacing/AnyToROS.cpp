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
    //for(uint j=0; j<maxPorts; j++) {
	outports_D.resize(maxPorts);
	outports_D_msg.resize(maxPorts);
    //}
    
    return true;
}

bool AnyToROS::startHook()
{
    return true;
}

void AnyToROS::AddDoublesToROS(int N, ints ids, string portname)
{
	// Check
	if(n_ports_D >= maxPorts) {
        log(Error) << "AnyToROS::AddDoublesToROS: Could not add doubles to ROS. Already to many ports: " << n_ports_D << " added. maxPorts: " << maxPorts << "!" << endlog();
	}
	if(ids.size() > N) {
        log(Error) << "AnyToROS::AddDoublesToROS: Could not add doubles to ROS. Since the length of vector ids: " << ids.size() << ", is longer than the total number of joints: " << N << "!" << endlog();
	}
	
	// Init
	n_ports_D++;
	ids_D[n_ports_D-1].resize(ids.size());
	input_D[n_ports_D-1].resize(ids.size());
	output_D_msg[n_ports_D-1].resize(ids.size());
	for(uint j=0; j<ids.size(); j++) {
		ids_D[n_ports_D-1][j]=ids[j];
		input_D[n_ports_D-1][j]=0.0;		
	}
	for(uint j=0; j<N; j++) {
		output_D_msg[n_ports_D-1][j].data = 0.0;
	}
	
	
	
	// Add ports
	if (n_ports_D==1) {
		addPort( "in_"+portname+"Ev", inports_D[n_ports_D-1]);
	} else {
		addPort( "in_"+portname, inports_D[n_ports_D-1]);
	}	
	
	for(uint j=0; j<N; j++) {	
		addPort( "out_"+portname+"_"+to_string(j+1), outports_D[n_ports_D-1][j] );
		addPort( "outmsg_"+portname+"_"+to_string(j+1), outports_D_msg[n_ports_D-1][j] );
	}
	
}

void AnyToROS::updateHook()
{
	// Doubles to ROS
	for(uint i=0; i<n_ports_D; i++) {
		if ( inports_D[i].read(input_D[i]) == NewData ) {
			for(uint k=0; k<ids_D[i].size(); k++) {
				// fill in the output_D_msg
				output_D_msg[i][k].data = input_D[i][ids_D[i][k]];
				// write to msg port
				outports_D_msg[i][k].write(output_D_msg[i][k]);
				// write to normal port
				outports_D[i][k].write(output_D_msg[i][k].data);
			}
		}
	}

}

ORO_CREATE_COMPONENT(ROS::AnyToROS)

//loadComponent("AnyToROS","ROS::AnyToROS")
//setActivity("AnyToROS",0.0,HighestPriority,ORO_SCHED_RT)
//AnyToROS.start
//AnyToROS.AddDoublesToROS(8, 8,"pos")
//AnyToROS.AddDoublesToROS(8, 8,"ref")
//AnyToROS.AddDoublesToROS(8, 8,"err")
//AnyToROS.AddDoublesToROS(8, 8,"eff")
//AnyToROS.AddDoublesToROS(8, 8,"for")
