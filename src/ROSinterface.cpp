#include "ROSinterface.hpp"

using namespace std;
using namespace RTT;
using namespace ROSINTERFACE;

ROSinterface::ROSinterface(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("AddDoubles2ROS", &ROSinterface::AddDoubles2ROS, this, OwnThread)
		.doc("Add a doubles 2 ROS")
		.arg("size","size of the doubles 2 ROS");
    addOperation("AddBool2ROS", &ROSinterface::AddBool2ROS, this, OwnThread)
		.doc("Add a bool 2 ROS");
    addOperation("AddJointstate2ROS", &ROSinterface::AddJointstate2ROS, this, OwnThread)
		.doc("Add a JointState 2 ROS")
		.arg("size","size of the JointState 2 ROS");
}

ROSinterface::~ROSinterface(){}

bool ROSinterface::configureHook()
{
	n_doubles_2ROS = 0;
	n_bool_2ROS = 0;
	n_jointstate_2ROS = 0;
}

bool ROSinterface::startHook()
{
	
}

void ROSinterface::updateHook()
{
	//! Doubles 2 ROS
	for( uint j = 0; j < n_doubles_2ROS; j++ ) { 
		// read input
		inports_D[j].read(input_D[j]);
		// copy input data to output structure
		for(uint i = 0; i < input_D[j].size(); ++i) {
			output_D[j].data[i] = input_D[j][i];
		}			
		// write output data to output port
		outports_D[j].write(output_D[j]);
	}
	
	//! Digital 2 ROS
	for( uint j = 0; j < n_bool_2ROS; j++ ) { 
		// read input
		inports_B[j].read(input_B[j]);
		// copy input data to output structure
		output_B[j].data = input_B[j];
		// write output data to output port
		outports_B[j].write(output_B[j]);
	}
	
	//! Jointstate 2 ROS
	// Read a doubles inport and put it's content into a jointstate message
	// Aggregate all jointstate messages and publish them on a ros topic
}

void ROSinterface::AddDoubles2ROS(uint SIZE)
{
	// Checks
	if (n_doubles_2ROS == MAX_PORTS) {
		log(Error) << "ROSinterface::AddDoubles2ROS: Could not add doubles 2 ROS. There are already 10 doubles signals going out!" << endlog();
		return;
	}
	if (SIZE <= 0) {
		log(Error) << "ROSinterface::AddDoubles2ROS: Could not add doubles 2 ROS. Size: " << SIZE << " should be larger than zero!" << endlog();
		return;
	}
	
	n_doubles_2ROS++;
	
	// Generate output structure  
	std_msgs::MultiArrayDimension dim;
	dim.size = SIZE;
	dim.label = "DoublesToRos"+to_string(n_doubles_2ROS);
	output_D[n_doubles_2ROS-1].layout.dim.push_back(dim);	
	
	// Resize in/output
	input_D[n_doubles_2ROS-1].resize(SIZE);
	output_D[n_doubles_2ROS-1].data.resize(SIZE);
	
	// Add in and output ports
	addPort( "DoublesToRos"+to_string(n_doubles_2ROS)+"_in", inports_D[n_doubles_2ROS-1] );
	addPort( "DoublesToRos"+to_string(n_doubles_2ROS)+"_out", outports_D[n_doubles_2ROS-1] );
	
    log(Warning) << "ROSinterface::AddDoubles2ROS: Added Doubles 2 ROS with size: " << SIZE << "!" << endlog();	
}

void ROSinterface::AddBool2ROS()
{
	// Checks
	if (n_bool_2ROS == MAX_PORTS) {
		log(Error) << "ROSinterface::AddDigital2ROS: Could not add bool 2 ROS. There are already 10 doubles signals going out!" << endlog();
		return;
	}
	
	n_bool_2ROS++;
	
	// Add in and output ports
	addPort( "BoolToRos"+to_string(n_bool_2ROS)+"_in", inports_B[n_bool_2ROS-1] );
	addPort( "BoolToRos"+to_string(n_bool_2ROS)+"_out", outports_B[n_bool_2ROS-1] );
	
    log(Warning) << "ROSinterface::AddDigital2ROS: Added Bool 2 ROS!" << endlog();	
}

void ROSinterface::AddJointstate2ROS(uint SIZE)
{
	// Checks
	
	// add 
	n_jointstate_2ROS++;
}

ORO_CREATE_COMPONENT(ROSINTERFACE::ROSinterface)
