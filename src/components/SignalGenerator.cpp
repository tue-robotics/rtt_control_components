#include "SignalGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALGENERATOR;

SignalGenerator::SignalGenerator(const string& name) : TaskContext(name, PreOperational)
{
	//! adding outputs
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
		
	//! Editing outputs
	addOperation("ClearSignal", &SignalGenerator::ClearSignal, this, OwnThread)
		.doc("Clears a signal to 0")
		.arg("type","Type of output ['Analog','Digital','Encoder']")
		.arg("id","ID number of that signal type");
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
		output_E_msgs[j].value = output_property_E[j];
		outports_E_msg[j].write(output_E_msgs[j]);
	}
	
	return;
}

void SignalGenerator::AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE)
{
	//! Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add analog signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_analog_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add analog signal. There are already 10 analog signals going out!" << endlog();
		return;
	}
	
	//! Update n_analog_signal counter and bools analog_message 
	n_analog_signal++;
	analog_message[n_analog_signal-1] = ANALOG_MESSAGE;

	//! Resize and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_A[n_analog_signal-1].resize(VECTOR_SIZE);
	addProperty( "A"+to_string(n_analog_signal)+"values", output_property_A[n_analog_signal-1] );		
	for( uint i = 0; i < output_property_A[n_analog_signal-1].size(); i++ ) {
		output_property_A[n_analog_signal-1][i] = DEFAULT_VALUES[i];
	}
	
	//! Init output
	if (ANALOG_MESSAGE) {
		output_A_msgs[n_analog_signal-1].values.assign(VECTOR_SIZE, 0.0);
	} else {
		output_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	}

	//! Add port
	if (ANALOG_MESSAGE) {
		addPort( "analogOut"+to_string(n_analog_signal), outports_A_msg[n_analog_signal-1] ).doc("Analog outport "+to_string(n_analog_signal)+" <AnalogMsg>");
		log(Warning) << "SignalGenerator::Adding AnalogMsg signal " << n_analog_signal << " with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "analogOut"+to_string(n_analog_signal), outports_A[n_analog_signal-1] ).doc("Analog outport "+to_string(n_analog_signal)+" <doubles>");
		log(Warning) << "SignalGenerator::Adding doubles signal  " << n_analog_signal << "  with size: " << VECTOR_SIZE << "!" << endlog();
	}

	return;
}

void SignalGenerator::AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool DIGITAL_MESSAGE)
{
	//! Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddDigitalSignal: Could not add digital signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_digital_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add digital signal. There are already 10 digital signals going out!" << endlog();
		return;
	}	
	
	//! Update n_digital_signal counter and bools digital_message 
	n_digital_signal++;
	digital_message[n_digital_signal-1] = DIGITAL_MESSAGE;
	
	//! Resize output_property_D and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_D[n_digital_signal-1].resize(VECTOR_SIZE);
	addProperty( "D"+to_string(n_digital_signal)+"values", output_property_D[n_digital_signal-1]);		
	for( uint i = 0; i < output_property_D[n_digital_signal-1].size(); i++ ) {
		output_property_D[n_digital_signal-1][i] = DEFAULT_VALUES[i];
	}

	//! Resize actual outputs	
	if (DIGITAL_MESSAGE) {
		output_D_msgs[n_digital_signal-1].values.assign(VECTOR_SIZE, 0);
	} else {
		output_D[n_digital_signal-1].assign(VECTOR_SIZE, 0);
	}

	//! Add port
	if (DIGITAL_MESSAGE) {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D_msg[n_digital_signal-1] ).doc("Digital outport "+to_string(n_digital_signal)+" <DigitalMsg>");
		log(Warning) << "SignalGenerator::Adding DigitalMsg signal " << n_digital_signal << " with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D[n_digital_signal-1] ).doc("Digital outport "+to_string(n_digital_signal)+" <ints>");
		log(Warning) << "SignalGenerator::Adding ints signal " << n_digital_signal << "  with size: " << VECTOR_SIZE << "!" << endlog();
	}

	return;
}

void SignalGenerator::AddEncoderSignal(double DEFAULT_VALUE)
{
	//! Check
	if (n_encoder_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddEncoderSignal: Could not add encoder signal. There are already 10 encoder signals going out!" << endlog();
		return;
	}
		
	//! Update global parameters
	n_encoder_signal++;
	
	//! Resize and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	addProperty( "E"+to_string(n_encoder_signal)+"value", output_property_E[n_encoder_signal-1] );		
	output_property_E[n_encoder_signal-1] = DEFAULT_VALUE;
	
	//! Add port
	addPort( "encoderOut"+to_string(n_encoder_signal), outports_E_msg[n_encoder_signal-1] );
	log(Warning) << "SignalGenerator::Adding EncoderMsg Signal!" << endlog();
	
	return;
}

void SignalGenerator::ClearSignal(string TYPE, int ID)
{
	//! Check
	if( ID <= 0 || ID > MAX_PORTS) {
		log(Error) << "SignalGenerator::ClearSignal: Could not clear signal. Invalid ID: " << ID << ".  1 <= ID <= " << MAX_PORTS << "!" << endlog();
		return;
	}

	if (TYPE == "Analog" || TYPE == "analog") {
		for( uint i = 0; i < output_property_A[ID-1].size(); i++ ) {
			output_property_A[ID-1][i] = 0.0;
		}
	} else if (TYPE == "Digital" || TYPE == "digital") {
		for( uint i = 0; i < output_property_D[ID-1].size(); i++ ) {
			output_property_D[ID-1][i] = 0;
		}
	} else if (TYPE == "Encoder" || TYPE == "encoder") {
		output_property_E[ID-1] = 0;
	} else {
		log(Error) << "SignalGenerator::ClearSignal: Could not clear signal. Wrong type: " << TYPE << ". Should be on of the following ['Analog','Digital','Encoder']!" << endlog();
		return;
	}

	return;
}

void SignalGenerator::CalculateRamp()
{

	return;
}



ORO_CREATE_COMPONENT(SIGNALGENERATOR::SignalGenerator)
