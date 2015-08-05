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
	addOperation("AddRamp", &SignalGenerator::AddRamp, this, OwnThread)
		.doc("Add a ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("start","array of doubles specifying the start value of the ramp")
		.arg("slope","array of doubles specifying the slope of the ramp")
		.arg("maximum","array of doubles specifying the maximum output");
	addOperation("AddNoise", &SignalGenerator::AddNoise, this, OwnThread)
		.doc("Add a ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("mean","array of doubles specifying the start value of the ramp")
		.arg("variance","array of doubles specifying the slope of the ramp");
	addOperation("AddSine", &SignalGenerator::AddSine, this, OwnThread)
		.doc("Add a ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("amplitude","array of doubles specifying the start value of the ramp")
		.arg("frequency","array of doubles specifying the slope of the ramp");
	addOperation("AddStep", &SignalGenerator::AddStep, this, OwnThread)
		.doc("Add a ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("steptime","array of doubles specifying the start value of the ramp")
		.arg("finalvalue","array of doubles specifying the slope of the ramp");
}

SignalGenerator::~SignalGenerator(){}

bool SignalGenerator::configureHook()
{
	//! Init
	n_analog_signal = 0;
	n_digital_signal = 0;
	n_encoder_signal = 0;
	
	for( uint l = 0; l < MAX_PORTS; l++ ) {
		ramp_status[l] = false;
		noise_status[l] = false;
		sine_status[l] = false;
		step_status[l] = false;
	}
	
	TS = getPeriod();
	
	return true;
}

bool SignalGenerator::startHook(){}

void SignalGenerator::updateHook()
{    
	// Analog
	CalculateRamp();
	CalculateNoise();
	CalculateSine();
	CalculateStep();

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
	
	CalculateRamp();
	
	return;
}

//! Functions to add outputs
void SignalGenerator::AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE)
{
	// Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add analog signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_analog_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add analog signal. There are already 10 analog signals going out!" << endlog();
		return;
	}
	
	// Update n_analog_signal counter and bools analog_message 
	n_analog_signal++;
	analog_message[n_analog_signal-1] = ANALOG_MESSAGE;

	// Resize and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_A[n_analog_signal-1].resize(VECTOR_SIZE);
	addProperty( "A"+to_string(n_analog_signal)+"values", output_property_A[n_analog_signal-1] );		
	for( uint i = 0; i < output_property_A[n_analog_signal-1].size(); i++ ) {
		output_property_A[n_analog_signal-1][i] = DEFAULT_VALUES[i];
	}
	
	// Init output
	if (ANALOG_MESSAGE) {
		output_A_msgs[n_analog_signal-1].values.assign(VECTOR_SIZE, 0.0);
	} else {
		output_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	}

	// Add port
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
	// Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddDigitalSignal: Could not add digital signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_digital_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add digital signal. There are already 10 digital signals going out!" << endlog();
		return;
	}	
	
	// Update n_digital_signal counter and bools digital_message 
	n_digital_signal++;
	digital_message[n_digital_signal-1] = DIGITAL_MESSAGE;
	
	// Resize output_property_D and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_property_D[n_digital_signal-1].resize(VECTOR_SIZE);
	addProperty( "D"+to_string(n_digital_signal)+"values", output_property_D[n_digital_signal-1]);		
	for( uint i = 0; i < output_property_D[n_digital_signal-1].size(); i++ ) {
		output_property_D[n_digital_signal-1][i] = DEFAULT_VALUES[i];
	}

	// Resize actual outputs	
	if (DIGITAL_MESSAGE) {
		output_D_msgs[n_digital_signal-1].values.assign(VECTOR_SIZE, 0);
	} else {
		output_D[n_digital_signal-1].assign(VECTOR_SIZE, 0);
	}

	// Add port
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
	// Check
	if (n_encoder_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddEncoderSignal: Could not add encoder signal. There are already 10 encoder signals going out!" << endlog();
		return;
	}
		
	// Update global parameters
	n_encoder_signal++;
	
	// Resize and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	addProperty( "E"+to_string(n_encoder_signal)+"value", output_property_E[n_encoder_signal-1] );		
	output_property_E[n_encoder_signal-1] = DEFAULT_VALUE;
	
	// Add port
	addPort( "encoderOut"+to_string(n_encoder_signal), outports_E_msg[n_encoder_signal-1] );
	log(Warning) << "SignalGenerator::Adding EncoderMsg Signal!" << endlog();
	
	return;
}

//! Functions to add signal sources to outputs
void SignalGenerator::ClearSignal(string TYPE, int ID)
{
	// Check
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

void SignalGenerator::AddRamp(int ID, doubles STARTVALUE, doubles SLOPE, doubles ENDVALUE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddRamp: Could not add ramp. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( STARTVALUE.size() != output_property_A[ID-1].size() || SLOPE.size() != output_property_A[ID-1].size() || ENDVALUE.size() != output_property_A[ID-1].size() ) {
		log(Error) << "SignalGenerator::AddRamp: Could not add ramp. Invalid size of STARTVALUE, SLOPE or ENDVALUE variable. Should all be of size : " << output_property_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	ramp_status[ID-1] = true;
	
	// Resize
	ramp_endvalue[ID-1].resize(output_property_A[ID-1].size());
	ramp_slope[ID-1].resize(output_property_A[ID-1].size());
	
	// Set values of ramp_endvalue and ramp_slope and update output_property_A to start value
	for( uint i = 0; i < output_property_A[ID-1].size(); i++ ) {
		ramp_endvalue[ID-1][i] = ENDVALUE[i];
		ramp_slope[ID-1][i] = SLOPE[i];
		output_property_A[ID-1][i] = STARTVALUE[i];
	}
	
	log(Warning) << "SignalGenerator::AddRamp: Succesfully added Ramp!" << endlog();
}

void SignalGenerator::AddNoise(int ID, doubles MEAN, doubles VARIANCE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddNoise: Could not add noise. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( MEAN.size() != output_property_A[ID-1].size() || VARIANCE.size() != output_property_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddNoise: Could not add noise. Invalid size of MEAN or VARIANCE variable. Should all be of size : " << output_property_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	noise_status[ID-1] = true;
	
	log(Warning) << "SignalGenerator::AddNoise: Succesfully added Noise!" << endlog();
}

void SignalGenerator::AddSine(int ID, doubles AMPLITUDE, doubles FREQUENCY)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddSine: Could not add sine. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( AMPLITUDE.size() != output_property_A[ID-1].size() || FREQUENCY.size() != output_property_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddSine: Could not add sine. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_property_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	sine_status[ID-1] = true;
	
	log(Warning) << "SignalGenerator::AddSine: Succesfully added Sine!" << endlog();
}

void SignalGenerator::AddStep(int ID, doubles STEPTIME, doubles FINALVALUE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddStep: Could not add step. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( STEPTIME.size() != output_property_A[ID-1].size() || FINALVALUE.size() != output_property_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddStep: Could not add step. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_property_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	step_status[ID-1] = true;	
	
	log(Warning) << "SignalGenerator::AddStep: Succesfully added Step!" << endlog();
}

//! Internal calculation functions
void SignalGenerator::CalculateRamp()
{
	// Analog
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (ramp_status[j]) {
			for( uint i = 0; i < output_property_A[j].size(); i++ ) {
				if (abs(output_property_A[j][i]) <= abs(ramp_endvalue[j][i])) {
					output_property_A[j][i] += ramp_slope[j][i]*TS;
				}
			}
		}
	}

	return;
}

void SignalGenerator::CalculateNoise()
{
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (noise_status[j]) {
			log(Warning) << "SignalGenerator::CalculateNoise: Calculationg Noise!" << endlog();
		}
	}
	
	return;
}

void SignalGenerator::CalculateSine()
{
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (sine_status[j]) {
			log(Warning) << "SignalGenerator::CalculateSine: Calculationg Sine!" << endlog();
		}
	}

	return;
}

void SignalGenerator::CalculateStep()
{
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (step_status[j]) {	
			log(Warning) << "SignalGenerator::CalculateStep: Calculationg Step!" << endlog();
		}
	}
	
	return;
}

ORO_CREATE_COMPONENT(SIGNALGENERATOR::SignalGenerator)
