#include "SignalGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALGENERATOR;

SignalGenerator::SignalGenerator(const string& name) : TaskContext(name, PreOperational)
{
	//! Operations
    addOperation("AddConstantSignal", &SignalGenerator::AddConstantSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("output_values","Array containing the output values")
		.arg("analog_message","Boolean specifying wether the output is published using a soem_beckhoff::AnalogMsg");
}
SignalGenerator::~SignalGenerator(){}

bool SignalGenerator::configureHook()
{
	//! Init
	n_constant_signal = 0;
	
	return true;
}

bool SignalGenerator::startHook(){}

void SignalGenerator::updateHook()
{    
	for( uint j = 0; j < n_constant_signal; j++ ) {
		if (analog_message[j]) {
			outports_A_msg[j].write(output_A_msgs[j]);
		} else {
			outports_A[j].write(output_A[j]);
		}	
	}
	return;
}

void SignalGenerator::AddConstantSignal(uint VECTOR_SIZE, doubles OUTPUT_VALUES, bool ANALOG_MESSAGE)
{
	//! Input property checks
	if (VECTOR_SIZE != OUTPUT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddConstantSignal: Could not add ConstantSignal. The size of OUTPUT_VALUES: " << OUTPUT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	
	//! Resizing
	analog_message.resize((n_constant_signal+1));
	if (ANALOG_MESSAGE) {
		output_A_msgs.resize((n_constant_signal+1));
		output_A_msgs[n_constant_signal].values.assign(VECTOR_SIZE, 0.0);
	} else {
		output_A.resize((n_constant_signal+1));
		output_A[n_constant_signal].assign(VECTOR_SIZE, 0.0);
	}
	
	//! Generate Output
	for( uint i = 0; i < VECTOR_SIZE; i++ ) {
		if (ANALOG_MESSAGE) {
			output_A_msgs[n_constant_signal].values[i] = OUTPUT_VALUES[i];
		} else {
			output_A[n_constant_signal][i] = OUTPUT_VALUES[i];
		}
	}
	
	//! Add port
	if (ANALOG_MESSAGE) {
		addPort( "constantOut"+to_string(n_constant_signal+1), outports_A_msg[n_constant_signal] );
		log(Warning) << "SignalGenerator::Adding AnalogMsg ConstantSignal with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "constantOut"+to_string(n_constant_signal+1), outports_A[n_constant_signal] );
		log(Warning) << "SignalGenerator::Adding doubles ConstantSignal with size: " << VECTOR_SIZE << "!" << endlog();
	}
	
	//! Update global parameters
	analog_message[n_constant_signal] = ANALOG_MESSAGE;
	n_constant_signal++;
	
	return;
}


ORO_CREATE_COMPONENT(SIGNALGENERATOR::SignalGenerator)
