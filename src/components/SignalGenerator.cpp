#include "SignalGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALGENERATOR;

SignalGenerator::SignalGenerator(const string& name) : TaskContext(name, PreOperational)
{
	//! Operations
    addOperation("AddAnalogSignal", &SignalGenerator::AddAnalogSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the output values")
		.arg("analog_message","Boolean specifying wether the output is published using a soem_beckhoff::AnalogMsg");
    addOperation("AddDigitalSignal", &SignalGenerator::AddDigitalSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the output values")
		.arg("digital_message","Boolean specifying wether the output is published using a soem_beckhoff::DigitalMsg");
    addOperation("AddEncoderSignal", &SignalGenerator::AddEncoderSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("default_value","Array containing the default value");
}

SignalGenerator::~SignalGenerator(){}

bool SignalGenerator::configureHook()
{
	//! Init
	n_analog_signal = 0;
	n_digital_signal = 0;
	n_encoder_signal = 0;
	
	return true;
}

bool SignalGenerator::startHook(){}

void SignalGenerator::updateHook()
{    
	// Analog
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (analog_message[j]) {
			for( uint i = 0; i < output_property_A[j].size(); i++ ) {
				output_A_msgs[j].values[i] = output_property_A[j][i];
			}
			outports_A_msg[j].write(output_A_msgs[j]);
		} else {
			for( uint i = 0; i < output_property_A[j].size(); i++ ) {
				output_A[j][i] = output_property_A[j][i];
			}
			outports_A[j].write(output_A[j]);
		}	
	}
	// Digital
	for( uint j = 0; j < n_digital_signal; j++ ) {
		if (digital_message[j]) {
			for( uint i = 0; i < output_property_D[j].size(); i++ ) {
				output_D_msgs[j].values[i] = (int) output_property_D[j][i];
			}
			outports_D_msg[j].write(output_D_msgs[j]);
		} else {
			for( uint i = 0; i < output_property_D[j].size(); i++ ) {
				output_D[j][i] = (int) output_property_D[j][i];
			}
			outports_D[j].write(output_D[j]);
		}	
	}
	// Encoder
	for( uint j = 0; j < n_encoder_signal; j++ ) {
		outports_E_msg[j].write(output_E_msgs[j]);
	}
	
	return;
}

void SignalGenerator::AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE)
{
	//! Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add ConstantSignal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	
	//! Update n_analog_signal counter and bools analog_message 
	n_analog_signal++;
	analog_message.resize(n_analog_signal);
	analog_message[n_analog_signal-1] = ANALOG_MESSAGE;

	//! Resize output_property_A and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_A.resize(n_analog_signal);
	output_property_A[n_analog_signal-1].resize(VECTOR_SIZE);
	addProperty( "A"+to_string(n_analog_signal)+"values", output_property_A[n_analog_signal-1] );		
	for( uint i = 0; i < output_property_A[n_analog_signal-1].size(); i++ ) {
		output_property_A[n_analog_signal-1][i] = DEFAULT_VALUES[i];
	}
	
	//! Resize actual outputs	
	if (ANALOG_MESSAGE) {
		output_A_msgs.resize(n_analog_signal);
		output_A_msgs[n_analog_signal-1].values.assign(VECTOR_SIZE, 0.0);
	} else {
		output_A.resize(n_analog_signal);
		output_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	}

	//! Add port
	if (ANALOG_MESSAGE) {
		addPort( "analogOut"+to_string(n_analog_signal), outports_A_msg[n_analog_signal-1] );
		log(Warning) << "SignalGenerator::Adding AnalogMsg signal with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "analogOut"+to_string(n_analog_signal), outports_A[n_analog_signal-1] );
		log(Warning) << "SignalGenerator::Adding doubles signal with size: " << VECTOR_SIZE << "!" << endlog();
	}

	return;
}

void SignalGenerator::AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool DIGITAL_MESSAGE)
{
	//! Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddDigitalSignal: Could not add ConstantSignal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	
	//! Update n_digital_signal counter and bools digital_message 
	n_digital_signal++;
	digital_message.resize(n_digital_signal);
	digital_message[n_digital_signal-1] = DIGITAL_MESSAGE;
	
	//! Resize output_property_D and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_D.resize(n_digital_signal);
	output_property_D[n_digital_signal-1].resize(VECTOR_SIZE);
	addProperty( "D"+to_string(n_digital_signal)+"values", output_property_D[n_digital_signal-1]);		
	for( uint i = 0; i < output_property_D[n_digital_signal-1].size(); i++ ) {
		output_property_D[n_digital_signal-1][i] = DEFAULT_VALUES[i];
	}

	//! Resize actual outputs	
	if (DIGITAL_MESSAGE) {
		output_D_msgs.resize(n_digital_signal);
		output_D_msgs[n_digital_signal-1].values.assign(VECTOR_SIZE, 0);
	} else {
		output_D.resize(n_digital_signal);
		output_D[n_digital_signal-1].assign(VECTOR_SIZE, 0);
	}

	//! Add port
	if (DIGITAL_MESSAGE) {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D_msg[n_digital_signal-1] );
		log(Warning) << "SignalGenerator::Adding DigitalMsg signal with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D[n_digital_signal-1] );
		log(Warning) << "SignalGenerator::Adding ints signal with size: " << VECTOR_SIZE << "!" << endlog();
	}

	return;
}

void SignalGenerator::AddEncoderSignal(double DEFAULT_VALUE)
{	
	//! Resizing
	output_E_msgs.resize((n_encoder_signal+1));
	
	//! Generate Output
	output_E_msgs[n_encoder_signal].value = DEFAULT_VALUE;
	
	//! Update global parameters
	n_encoder_signal++;
	
	//! Add port
	addPort( "encoderOut"+to_string(n_encoder_signal), outports_E_msg[n_encoder_signal-1] );
	log(Warning) << "SignalGenerator::Adding EncoderMsg Signal!" << endlog();
	
	return;
}

ORO_CREATE_COMPONENT(SIGNALGENERATOR::SignalGenerator)
