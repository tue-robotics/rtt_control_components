#include "ROSwrite.hpp"

using namespace std;
using namespace RTT;
using namespace ROSWRITE;

ROSwrite::ROSwrite(const string& name) : TaskContext(name, PreOperational)
{
	//! Properties
	addProperty( "bodypart_names",     	bodypart_names     ).doc("Names of bodyparts");	
	
	//! Operations
	addOperation("AddAnalogWrite", &ROSwrite::AddAnalogWrite, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("PARTNAME","String specifying the name of the part")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry");
    addOperation("AddDigitalWrite", &ROSwrite::AddDigitalWrite, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("PARTNAME","String specifying the name of the part")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry");
}
ROSwrite::~ROSwrite(){}

bool ROSwrite::configureHook()
{
	//! init
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	goodToGO = false;
	
	return true;
}

bool ROSwrite::startHook()
{
	if (bodypart_names.size() <= 0) {
		log(Error) << "ROSwrite::startHook: Could not start ROSwrite. First add a list of the bodyparts that will be added. (at least one) " << bodypart_names.size() << "!" << endlog();
		return false;
	}
	
	return true;
}

void ROSwrite::updateHook()
{
	// Functions checks the connections, (It does this once after X seconds)
	CheckAllConnections();
	
	// Reads inputs, this extracts data from ethercat slaves
	ReadInputs();
	
	// Function to write the output calculated onto the output port
	WriteOutputs();
	
	return;
}

void ROSwrite::AddAnalogWrite(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	
}

void ROSwrite::AddDigitalWrite(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	
}

void ROSwrite::CheckAllConnections()
{
	//! 8 seconds after boot, Check all connections
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 10.0)) {
		goodToGO = true;
	}
}

void ROSwrite::ReadInputs()
{
	
}

void ROSwrite::WriteOutputs()
{
	
}

ORO_CREATE_COMPONENT(ROSWRITE::ROSwrite)
