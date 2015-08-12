#include "SOEMdummy.hpp"

using namespace std;
using namespace RTT;
using namespace SOEMDUMMY;

SOEMdummy::SOEMdummy(const string& name) : TaskContext(name, PreOperational)
{
	//! Adding Sinks	
	addOperation("AddAnalogSink", &SOEMdummy::AddAnalogSink, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","uint specifying size of the port")
		.arg("name","string specifying port name");
    addOperation("AddDigitalSink", &SOEMdummy::AddDigitalSink, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","uint specifying size of the port")
		.arg("name","string specifying port name");
	
	//! Adding outputs
    addOperation("AddAnalogSignal", &SOEMdummy::AddAnalogSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the output values")
		.arg("name","string specifying port name");
    addOperation("AddEncoderSignal", &SOEMdummy::AddEncoderSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the default values")
		.arg("name","string specifying port name");
    addOperation("AddDigitalSignal", &SOEMdummy::AddDigitalSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the output values")
		.arg("name","string specifying port name");
}

SOEMdummy::~SOEMdummy(){}

bool SOEMdummy::configureHook()
{
	//! Init
	n_analog_sink = 0;
	n_digital_sink = 0;
	n_analog_signal = 0;
	n_digital_signal = 0;
	n_encoder_signal = 0;
	
	return true;
}

bool SOEMdummy::startHook(){}

void SOEMdummy::updateHook()
{    

	// Write Output
	WriteOutput();
	
	return;
}

//! Functions to add outputs
void SOEMdummy::AddAnalogSink(uint VECTOR_SIZE, string NAME)
{
	return;
}

void SOEMdummy::AddDigitalSink(uint VECTOR_SIZE, string NAME)
{
	return;
}

void SOEMdummy::AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME)
{
	// Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SOEMdummy::AddAnalogSignal: Could not add analog signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_analog_signal == MAX_PORTS) {
		log(Error) << "SOEMdummy::AddAnalogSignal: Could not add analog signal. There are already " << MAX_PORTS << " analog signals going out!" << endlog();
		return;
	}
	
	// Update n_analog_signal counter and bools analog_message 
	n_analog_signal++;

	// Resize output_A and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	addProperty( "A"+to_string(n_analog_signal)+"values", output_A[n_analog_signal-1] );		
	for( uint i = 0; i < VECTOR_SIZE; i++ ) {
		output_A[n_analog_signal-1][i] = DEFAULT_VALUES[i];
	}
	
	// Init output
	output_A_msgs[n_analog_signal-1].values.assign(VECTOR_SIZE, 0.0);

	// Add port
	addPort( "Slave_"+NAME, outports_A_msg[n_analog_signal-1] ).doc("Soem.Slave_"+NAME+" <AnalogMsg>");
	log(Warning) << "SOEMdummy::AddAnalogSignal: Adding AnalogMsg signal " << n_analog_signal << " with size: " << VECTOR_SIZE << "!" << endlog();

	return;
}

void SOEMdummy::AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME)
{
	return;
}

void SOEMdummy::AddEncoderSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME)
{
	return;
}

void SOEMdummy::WriteOutput()
{
	// Analog
	for( uint j = 0; j < n_analog_signal; j++ ) {
		for( uint i = 0; i < output_A[j].size(); i++ ) {
			output_A_msgs[j].values[i] = output_A[j][i];
		}
		outports_A_msg[j].write(output_A_msgs[j]);
	}
	
	return;
}

ORO_CREATE_COMPONENT(SOEMDUMMY::SOEMdummy)
