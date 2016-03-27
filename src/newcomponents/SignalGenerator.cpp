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
    addOperation("AddIntegerSignal", &SignalGenerator::AddIntegerSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Array containing the default values")
		.arg("default_values","Array containing the default values")
		.arg("encoder_message","Boolean specifying wether the output is published using a soem_beckhoff::EncoderMsg");
    addOperation("AddDigitalSignal", &SignalGenerator::AddDigitalSignal, this, OwnThread)
		.doc("Add a constant signal port")
		.arg("vector_size","Number of outputs of the particular port")
		.arg("default_values","Array containing the output values")
		.arg("digital_message","Boolean specifying wether the output is published using a soem_beckhoff::DigitalMsg");	
	
	//! Editing outputs
	// Analog
	addOperation("AddRamp_A", &SignalGenerator::AddRamp_A, this, OwnThread)
		.doc("Add a analog ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("slope","array of doubles specifying the slope of the ramp")
		.arg("maximum","array of doubles specifying the maximum output");
	addOperation("AddNoise_A", &SignalGenerator::AddNoise_A, this, OwnThread)
		.doc("Add a analog noise signal")
		.arg("id","Id number of the particular output")
		.arg("mean","array of doubles specifying the mean value of the noise")
		.arg("variance","array of doubles specifying the variance of the noise");
	addOperation("AddSine_A", &SignalGenerator::AddSine_A, this, OwnThread)
		.doc("Add a analog sine signal")
		.arg("id","Id number of the particular output")
		.arg("amplitude","array of doubles specifying the start value of the sine")
		.arg("frequency","array of doubles specifying the slope of the sine")
		.arg("phase","array of doubles specifying the phase of the sine");
	addOperation("AddStep_A", &SignalGenerator::AddStep_A, this, OwnThread)
		.doc("Add a analog step signal")
		.arg("id","Id number of the particular output")
		.arg("steptime","array of doubles specifying the time in seconds after the calling of the function when the step is done")
		.arg("stepvalue","array of doubles specifying the step increase value");
	
	// Digital
	addOperation("AddStep_D", &SignalGenerator::AddStep_D, this, OwnThread)
		.doc("Add a digital step signal")
		.arg("id","Id number of the particular output")
		.arg("steptime","array of doubles specifying the time in seconds after the calling of the function when the step is done")
		.arg("stepvalue","array of doubles specifying the step increase value");
	
	// Integer
	addOperation("AddRamp_I", &SignalGenerator::AddRamp_I, this, OwnThread)
		.doc("Add a integer ramp signal")
		.arg("id","Type of output ['Analog','Digital','Encoder']")
		.arg("slope","array of doubles specifying the slope of the ramp")
		.arg("maximum","array of doubles specifying the maximum output");
	addOperation("AddSine_I", &SignalGenerator::AddSine_I, this, OwnThread)
		.doc("Add a integer sine signal")
		.arg("id","Id number of the particular output")
		.arg("amplitude","array of doubles specifying the start value of the sine")
		.arg("frequency","array of doubles specifying the slope of the sine")
		.arg("phase","array of doubles specifying the phase of the sine");
	addOperation("AddStep_I", &SignalGenerator::AddStep_I, this, OwnThread)
		.doc("Add a integer step signal")
		.arg("id","Id number of the particular output")
		.arg("steptime","array of doubles specifying the time in seconds after the calling of the function when the step is done")
		.arg("stepvalue","array of doubles specifying the step increase value");
}

SignalGenerator::~SignalGenerator(){}

bool SignalGenerator::configureHook()
{
	//! Init
	n_analog_signal = 0;
	n_digital_signal = 0;
	n_integer_signal = 0;
	TS = getPeriod();
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	time = 0.0;
	
	for( uint l = 0; l < MAX_PORTS; l++ ) {
		ramp_status_A[l] = false;
		noise_status_A[l] = false;
		sine_status_A[l] = false;
		step_status_A[l] = false;
		step_status_D[l] = false;
		ramp_status_I[l] = false;
		sine_status_I[l] = false;
		step_status_I[l] = false;
	}
	
	return true;
}

bool SignalGenerator::startHook(){}

void SignalGenerator::updateHook()
{
	// Set output zero
	SetOutputZero();
	
	// Analog	
	CalculateRamp_A();
	CalculateNoise_A();
	CalculateSine_A();
	CalculateStep_A();
	
	// Digital
	CalculateStep_D();

	// Integer
	CalculateRamp_I();
	CalculateSine_I();
	CalculateStep_I();
	
	// Write Output	
	WriteOutput();
	
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
		log(Error) << "SignalGenerator::AddAnalogSignal: Could not add analog signal. There are already " << MAX_PORTS << " analog signals going out!" << endlog();
		return;
	}
	
	// Update n_analog_signal counter and bools analog_message 
	n_analog_signal++;
	analog_message[n_analog_signal-1] = ANALOG_MESSAGE;

	// Resize both intermediate outputs and add output_additive_A as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_nonadditive_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	output_additive_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	output_A[n_analog_signal-1].assign(VECTOR_SIZE, 0.0);
	addProperty( "A"+to_string(n_analog_signal)+"values", output_additive_A[n_analog_signal-1] );		
	for( uint i = 0; i < output_additive_A[n_analog_signal-1].size(); i++ ) {
		output_additive_A[n_analog_signal-1][i] = DEFAULT_VALUES[i];
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
		log(Info) << "SignalGenerator::AddAnalogSignal: Adding AnalogMsg signal " << n_analog_signal << " with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "analogOut"+to_string(n_analog_signal), outports_A[n_analog_signal-1] ).doc("Analog outport "+to_string(n_analog_signal)+" <doubles>");
		log(Info) << "SignalGenerator::AddAnalogSignal: Adding doubles signal  " << n_analog_signal << "  with size: " << VECTOR_SIZE << "!" << endlog();
	}
		
	return;
}

void SignalGenerator::AddIntegerSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ENCODER_MESSAGE)
{
	// Input property checks
	if (VECTOR_SIZE != DEFAULT_VALUES.size()) {
		log(Error) << "SignalGenerator::AddIntegerSignal: Could not add integer signal. The size of DEFAULT_VALUES: " << DEFAULT_VALUES.size() << " should match VECTOR_SIZE: " << VECTOR_SIZE << "!" << endlog();
		return;
	}
	if (n_integer_signal == MAX_PORTS) {
		log(Error) << "SignalGenerator::AddIntegerSignal: Could not add integer signal. There are already " << MAX_PORTS << " integer signals going out!" << endlog();
		return;
	}	
	if (ENCODER_MESSAGE && (VECTOR_SIZE != 1 || DEFAULT_VALUES.size() != 1) ) {
		log(Error) << "SignalGenerator::AddIntegerSignal: Could not add integer signal. EncoderMsg is asked while Vector size and or the size of default_values are not equal to 1!" << endlog();
		return;
	}	
	
	// Update n_integer_signal counter and bools integer_message 
	n_integer_signal++;
	integer_message[n_integer_signal-1] = ENCODER_MESSAGE;
	
	// Resize both intermediate outputs and add output_additive_A as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_nonadditive_I[n_integer_signal-1].assign(VECTOR_SIZE, 0.0);
	output_additive_I[n_integer_signal-1].resize(VECTOR_SIZE);
	output_I[n_integer_signal-1].resize(VECTOR_SIZE);
	addProperty( "I"+to_string(n_integer_signal)+"values", output_additive_I[n_integer_signal-1]);		
	for( uint i = 0; i < output_additive_I[n_integer_signal-1].size(); i++ ) {
		output_additive_I[n_integer_signal-1][i] = DEFAULT_VALUES[i];
	}

	// Resize actual outputs	
	if (!ENCODER_MESSAGE) {
		output_I[n_integer_signal-1].assign(VECTOR_SIZE, 0);
	}

	// Add port
	if (ENCODER_MESSAGE) {
		addPort( "integerOut"+to_string(n_integer_signal), outports_I_msg[n_integer_signal-1] ).doc("Integer outport "+to_string(n_integer_signal)+" <EncoderMsg>");
		log(Info) << "SignalGenerator::AddIntegerSignal: Adding EncoderMsg signal " << n_integer_signal << " with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "integerOut"+to_string(n_integer_signal), outports_I[n_integer_signal-1] ).doc("Integer outport "+to_string(n_integer_signal)+" <ints>");
		log(Info) << "SignalGenerator::AddIntegerSignal: Adding ints signal " << n_integer_signal << "  with size: " << VECTOR_SIZE << "!" << endlog();
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
		log(Error) << "SignalGenerator::AddDigitalSignal: Could not add digital signal. There are already " << MAX_PORTS << " digital signals going out!" << endlog();
		return;
	}
	if (!DIGITAL_MESSAGE && (VECTOR_SIZE != 1 || DEFAULT_VALUES.size() != 1) ) {
		log(Error) << "SignalGenerator::AddDigitalSignal: Could not add digital signal. No DIGITAL_MESSAGE is asked while Vector size and or the size of default_values are not equal to 1!" << endlog();
		return;
	}	
	
	// Update n_digital_signal counter and bools digital_message 
	n_digital_signal++;
	digital_message[n_digital_signal-1] = DIGITAL_MESSAGE;
	
	// Resize output_additive_D and add as property then assign DEFAULT_VALUES (This way upon runtime the property can be used to update)
	output_additive_D[n_digital_signal-1].assign(VECTOR_SIZE, 0.0);
	addProperty( "D"+to_string(n_digital_signal)+"values", output_additive_D[n_digital_signal-1]);		
	for( uint i = 0; i < output_additive_D[n_digital_signal-1].size(); i++ ) {
		output_additive_D[n_digital_signal-1][i] = DEFAULT_VALUES[i];
	}

	// Resize actual outputs	
	if (DIGITAL_MESSAGE) {
		output_D_msgs[n_digital_signal-1].values.assign(VECTOR_SIZE, 0);
	}

	// Add port
	if (DIGITAL_MESSAGE) {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D_msg[n_digital_signal-1] ).doc("Digital outport "+to_string(n_digital_signal)+" <DigitalMsg>");
		log(Info) << "SignalGenerator::Adding DigitalMsg signal " << n_digital_signal << " with size: " << VECTOR_SIZE << "!" << endlog();
	} else {
		addPort( "digitalOut"+to_string(n_digital_signal), outports_D[n_digital_signal-1] ).doc("Digital outport "+to_string(n_digital_signal)+" <bools>");
		log(Info) << "SignalGenerator::Adding bool signal " << n_digital_signal << "  of size 1!" << endlog();
	}

	return;
}

//! Functions to add signal sources to outputs
void SignalGenerator::AddRamp_A(int ID, doubles SLOPE, doubles ENDVALUE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddRamp_A: Could not add ramp. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( SLOPE.size() != output_A[ID-1].size() || ENDVALUE.size() != output_A[ID-1].size() ) {
		log(Error) << "SignalGenerator::AddRamp_A: Could not add ramp. Invalid size of SLOPE or ENDVALUE variable. Should all be of size : " << output_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	ramp_status_A[ID-1] = true;
	
	// Resize
	ramp_endvalue_A[ID-1].resize(output_A[ID-1].size());
	ramp_slope_A[ID-1].resize(output_A[ID-1].size());
	
	// Save ramp properties
	for( uint i = 0; i < output_A[ID-1].size(); i++ ) {
		ramp_endvalue_A[ID-1][i] = ENDVALUE[i];
		ramp_slope_A[ID-1][i] = SLOPE[i];
	}
	
	log(Info) << "SignalGenerator::AddRamp_A: Succesfully added ramp of size: " << ramp_endvalue_A[ID-1].size() << "!" << endlog();
	
	for( uint i = 0; i < ramp_endvalue_A[ID-1].size(); i++ ) {
		if( output_additive_A[ID-1][i] > ramp_endvalue_A[ID-1][i] ) {
			log(Warning) << "SignalGenerator::AddRamp_A: Note that for output: " << i+1 << ", the output already exceeded it's end value and hence this ramp is inactive!" << endlog();
		}
	}
}

void SignalGenerator::AddRamp_I(int ID, doubles SLOPE, doubles ENDVALUE)
{
	// Check
	if( ID <= 0 || ID > n_integer_signal) {
		log(Error) << "SignalGenerator::AddRamp_I: Could not add ramp. Invalid ID: " << ID << ".  1 <= ID <= " << n_integer_signal << "!" << endlog();
		return;
	}
	if( SLOPE.size() != output_I[ID-1].size() || ENDVALUE.size() != output_I[ID-1].size() ) {
		log(Error) << "SignalGenerator::AddRamp_I: Could not add ramp. Invalid size of SLOPE or ENDVALUE variable. Should all be of size : " << output_I[ID-1].size() << "!" << endlog();
		return;
	}

	// Set status
	ramp_status_I[ID-1] = true;
	
	// Resize
	ramp_endvalue_I[ID-1].resize(output_I[ID-1].size());
	ramp_slope_I[ID-1].resize(output_I[ID-1].size());
	
	// Save ramp properties
	for( uint i = 0; i < output_I[ID-1].size(); i++ ) {
		ramp_endvalue_I[ID-1][i] = ENDVALUE[i];
		ramp_slope_I[ID-1][i] = SLOPE[i];
	}
	
	log(Info) << "SignalGenerator::AddRamp_I: Succesfully added ramp of size: " << ramp_endvalue_I[ID-1].size() << "!" << endlog();
	
	for( uint i = 0; i < ramp_endvalue_I[ID-1].size(); i++ ) {
		if( ramp_endvalue_I[ID-1][i] > output_additive_I[ID-1][i] ) {
			log(Warning) << "SignalGenerator::AddRamp_I: Note that for output: " << i+1 << ", the output already exceeded it's end value and hence this ramp is inactive!" << endlog();
		}
	}
}

void SignalGenerator::AddNoise_A(int ID, doubles MEAN, doubles VARIANCE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddNoise_A: Could not add noise. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( MEAN.size() != output_A[ID-1].size() || VARIANCE.size() != output_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddNoise_A: Could not add noise. Invalid size of MEAN or VARIANCE variable. Should all be of size : " << output_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	noise_status_A[ID-1] = true;
	
	// Resize
	noise_mean_A[ID-1].resize(output_A[ID-1].size());
	noise_variance_A[ID-1].resize(output_A[ID-1].size());
	
	// Save noise properties
	for( uint i = 0; i < output_A[ID-1].size(); i++ ) {
		noise_mean_A[ID-1][i] = MEAN[i];
		noise_variance_A[ID-1][i] = VARIANCE[i];
	}
	
	log(Info) << "SignalGenerator::AddNoise_A: Succesfully added Noise!" << endlog();
}

void SignalGenerator::AddSine_A(int ID, doubles AMPLITUDE, doubles FREQUENCY, doubles PHASE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddSine_A: Could not add sine. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( AMPLITUDE.size() != output_A[ID-1].size() || FREQUENCY.size() != output_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddSine_A: Could not add sine. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	sine_status_A[ID-1] = true;
	
	// Resize
	sine_amplitude_A[ID-1].resize(output_A[ID-1].size());
	sine_frequency_A[ID-1].resize(output_A[ID-1].size());
	sine_phase_A[ID-1].resize(output_A[ID-1].size());
	
	// Save sine properties
	for( uint i = 0; i < output_A[ID-1].size(); i++ ) {
		sine_amplitude_A[ID-1][i] = AMPLITUDE[i];
		sine_frequency_A[ID-1][i] = FREQUENCY[i];
		sine_phase_A[ID-1][i] = PHASE[i];
	}
	
	log(Info) << "SignalGenerator::AddSine_A: Succesfully added Sine!" << endlog();
}

void SignalGenerator::AddSine_I(int ID, doubles AMPLITUDE, doubles FREQUENCY, doubles PHASE)
{
	// Check
	if( ID <= 0 || ID > n_integer_signal) {
		log(Error) << "SignalGenerator::AddSine_I: Could not add sine. Invalid ID: " << ID << ".  1 <= ID <= " << n_integer_signal << "!" << endlog();
		return;
	}
	if( AMPLITUDE.size() != output_I[ID-1].size() || FREQUENCY.size() != output_I[ID-1].size()) {
		log(Error) << "SignalGenerator::AddSine_I: Could not add sine. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_I[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	sine_status_I[ID-1] = true;
	
	// Resize
	sine_amplitude_I[ID-1].resize(output_I[ID-1].size());
	sine_frequency_I[ID-1].resize(output_I[ID-1].size());
	sine_phase_I[ID-1].resize(output_I[ID-1].size());
	
	// Save sine properties
	for( uint i = 0; i < output_I[ID-1].size(); i++ ) {
		sine_amplitude_I[ID-1][i] = AMPLITUDE[i];
		sine_frequency_I[ID-1][i] = FREQUENCY[i];
		sine_phase_I[ID-1][i] = PHASE[i];
	}
	
	log(Info) << "SignalGenerator::AddSine_I: Succesfully added Sine!" << endlog();
}

void SignalGenerator::AddStep_A(int ID, doubles STEPTIME, doubles STEPVALUE)
{
	// Check
	if( ID <= 0 || ID > n_analog_signal) {
		log(Error) << "SignalGenerator::AddStep_A: Could not add step. Invalid ID: " << ID << ".  1 <= ID <= " << n_analog_signal << "!" << endlog();
		return;
	}
	if( STEPTIME.size() != output_additive_A[ID-1].size() || STEPVALUE.size() != output_additive_A[ID-1].size()) {
		log(Error) << "SignalGenerator::AddStep_A: Could not add step. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_additive_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	step_status_A[ID-1] = true;	
	
	// Resize
	step_time_A[ID-1].resize(output_additive_A[ID-1].size());
	step_value_A[ID-1].resize(output_additive_A[ID-1].size());
	
	// Save step properties
	for( uint i = 0; i < output_additive_A[ID-1].size(); i++ ) {
		step_time_A[ID-1][i] = STEPTIME[i];
		step_value_A[ID-1][i] = STEPVALUE[i];
	}
	
	log(Info) << "SignalGenerator::AddStep_A: Succesfully added Step!" << endlog();
}

void SignalGenerator::AddStep_I(int ID, doubles STEPTIME, doubles STEPVALUE)
{
	// Check
	if( ID <= 0 || ID > n_integer_signal) {
		log(Error) << "SignalGenerator::AddStep_I: Could not add step. Invalid ID: " << ID << ".  1 <= ID <= " << n_integer_signal << "!" << endlog();
		return;
	}
	if( STEPTIME.size() != output_additive_I[ID-1].size() || STEPVALUE.size() != output_additive_I[ID-1].size()) {
		log(Error) << "SignalGenerator::AddStep_I: Could not add step. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_additive_I[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	step_status_I[ID-1] = true;	
	
	// Resize
	step_time_I[ID-1].resize(output_additive_I[ID-1].size());
	step_value_I[ID-1].resize(output_additive_I[ID-1].size());
	
	// Save step properties
	for( uint i = 0; i < output_additive_I[ID-1].size(); i++ ) {
		step_time_I[ID-1][i] = STEPTIME[i];
		step_value_I[ID-1][i] = STEPVALUE[i];
	}
	
	log(Info) << "SignalGenerator::AddStep_I: Succesfully added Step!" << endlog();
}

void SignalGenerator::AddStep_D(int ID, doubles STEPTIME, doubles STEPVALUE)
{
	// Check
	if( ID <= 0 || ID > n_digital_signal) {
		log(Error) << "SignalGenerator::AddStep_D: Could not add step. Invalid ID: " << ID << ".  1 <= ID <= " << n_digital_signal << "!" << endlog();
		return;
	}
	if( STEPTIME.size() != output_additive_D[ID-1].size() || STEPVALUE.size() != output_additive_D[ID-1].size()) {
		log(Error) << "SignalGenerator::AddStep_D: Could not add step. Invalid size of AMPLITUDE or FREQUENCY variable. Should all be of size : " << output_additive_D[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Set status
	step_status_D[ID-1] = true;	
	
	// Resize
	step_time_D[ID-1].resize(output_additive_D[ID-1].size());
	step_value_D[ID-1].resize(output_additive_D[ID-1].size());
	
	// Save step properties
	for( uint i = 0; i < output_additive_D[ID-1].size(); i++ ) {
		step_time_D[ID-1][i] = STEPTIME[i];
		step_value_D[ID-1][i] = STEPVALUE[i];
	}
	
	log(Info) << "SignalGenerator::AddStep_D: Succesfully added Step!" << endlog();
}

//! Calculate functions
void SignalGenerator::CalculateRamp_A()
{
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (ramp_status_A[j]) {
			for( uint i = 0; i < output_additive_A[j].size(); i++ ) {
				if (abs(output_additive_A[j][i]) < abs(ramp_endvalue_A[j][i])) {
					output_additive_A[j][i] += ramp_slope_A[j][i]*TS;
				}
			}
		}
	}
}

void SignalGenerator::CalculateRamp_I()
{
	for( uint j = 0; j < n_integer_signal; j++ ) {
		if (ramp_status_I[j]) {
			for( uint i = 0; i < output_additive_I[j].size(); i++ ) {
				if (abs(output_additive_I[j][i]) < abs(ramp_endvalue_I[j][i])) {
					output_additive_I[j][i] += ramp_slope_I[j][i]*TS;
				}
			}
		}
	}
}

void SignalGenerator::CalculateNoise_A()
{
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (noise_status_A[j]) {
			for( uint i = 0; i < output_additive_A[j].size(); i++ ) {
				output_nonadditive_A[j][i] += box_muller(noise_mean_A[j][i], noise_variance_A[j][i]);
			}
		}
	}
}

void SignalGenerator::CalculateSine_A()
{
	time = (os::TimeService::Instance()->getNSecs()*1e-9)-start_time;

	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (sine_status_A[j]) {
			for( uint i = 0; i < output_A[j].size(); i++ ) {
				output_nonadditive_A[j][i] = sine_amplitude_A[j][i] * sin(2*PI*sine_frequency_A[j][i]*time+sine_phase_A[j][i]);
			}
		}
	}
}

void SignalGenerator::CalculateSine_I()
{
	time = (os::TimeService::Instance()->getNSecs()*1e-9)-start_time;

	for( uint j = 0; j < n_integer_signal; j++ ) {
		if (sine_status_I[j]) {
			for( uint i = 0; i < output_I[j].size(); i++ ) {
				output_nonadditive_I[j][i] = sine_amplitude_I[j][i] * sin(2*PI*sine_frequency_I[j][i]*time+sine_phase_I[j][i]);
			}
		}
	}
}

void SignalGenerator::CalculateStep_A()
{
	time = (os::TimeService::Instance()->getNSecs()*1e-9)-start_time;
	
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (step_status_A[j]) {	
			for( uint i = 0; i < output_additive_A[j].size(); i++ ) {
				if (time >= step_time_A[j][i]) {
					output_additive_A[j][i] += step_value_A[j][i];
					step_value_A[j][i] = 0.0;
				}
			}
		}
	}
}

void SignalGenerator::CalculateStep_I()
{
	time = (os::TimeService::Instance()->getNSecs()*1e-9)-start_time;
	
	for( uint j = 0; j < n_integer_signal; j++ ) {
		if (step_status_I[j]) {	
			for( uint i = 0; i < output_additive_I[j].size(); i++ ) {
				
				if (time >= step_time_I[j][i]) {
					output_additive_I[j][i] += step_value_I[j][i];
					step_value_I[j][i] = 0.0;
				}
			}
		}
	}
}

void SignalGenerator::CalculateStep_D()
{
	time = (os::TimeService::Instance()->getNSecs()*1e-9)-start_time;
	
	for( uint j = 0; j < n_digital_signal; j++ ) {
		if (step_status_D[j]) {	
			for( uint i = 0; i < output_additive_D[j].size(); i++ ) {
				
				if (time >= step_time_D[j][i]) {
					output_additive_D[j][i] = (bool) step_value_D[j][i];
				}
			}
		}
	}
}

void SignalGenerator::SetOutputZero()
{
	// Analog
	for( uint j = 0; j < n_analog_signal; j++ ) {
		for( uint i = 0; i < output_A[j].size(); i++ ) {
			output_A[j][i] = 0.0;
			output_nonadditive_A[j][i] = 0.0;
		}
	}
	
	// Integer
	for( uint j = 0; j < n_integer_signal; j++ ) {
		for( uint i = 0; i < output_I[j].size(); i++ ) {
			output_I[j][i] = 0.0;
			output_nonadditive_I[j][i] = 0.0;
		}
	}
	
	return;
}

void SignalGenerator::WriteOutput()
{
	// Analog
	for( uint j = 0; j < n_analog_signal; j++ ) {
		if (analog_message[j]) {
			for( uint i = 0; i < output_additive_A[j].size(); i++ ) {
				output_A_msgs[j].values[i] = output_nonadditive_A[j][i]+output_additive_A[j][i];
			}
			outports_A_msg[j].write(output_A_msgs[j]);
		} else {
			for( uint i = 0; i < output_additive_A[j].size(); i++ ) {
				output_A[j][i] = output_nonadditive_A[j][i]+output_additive_A[j][i];
			}
			outports_A[j].write(output_A[j]);
		}
	}
	
	// Digital
	for( uint j = 0; j < n_digital_signal; j++ ) {
		if (digital_message[j]) {
			for( uint i = 0; i < output_additive_D[j].size(); i++ ) {
				output_D_msgs[j].values[i] = (bool) output_additive_D[j][i];
			}
			outports_D_msg[j].write(output_D_msgs[j]);
		} else {
			output_D[j] = (bool) output_additive_D[j][0];
			outports_D[j].write(output_D[j]);
		}
	}
	
	// Integer
	for( uint j = 0; j < n_integer_signal; j++ ) {
		if (integer_message[j]) {
			for( uint i = 0; i < output_additive_I[j].size(); i++ ) {
				output_I_msgs[j].value = (int) output_nonadditive_I[j][i] + (int) output_additive_I[j][i];
			}
			outports_I_msg[j].write(output_I_msgs[j]);
		} else {
			for( uint i = 0; i < output_additive_I[j].size(); i++ ) {
				output_I[j][i] = (int) output_nonadditive_I[j][i] + (int) output_additive_I[j][i];
			}
			outports_I[j].write(output_I[j]);
		}
	}
	
	return;
}

//! Support functions
double SignalGenerator::randomnumgen(double LOW, double HIGH)
{
	double range=(HIGH-LOW);
	double num = rand() * range / RAND_MAX + LOW ;
	
	return(num);
}

double SignalGenerator::box_muller(double M, double V)	
{								
	double x1, x2, w, y1;
	static double y2;
	static int use_last = 0;

	/* use value from previous call */
	if (use_last) {
		y1 = y2;
		use_last = 0;
	} else {
		do {
			x1 = 2.0 * randomnumgen(0.0,1.0) - 1.0;
			x2 = 2.0 * randomnumgen(0.0,1.0) - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( M + y1 * sqrt(V) );
}

ORO_CREATE_COMPONENT(SIGNALGENERATOR::SignalGenerator)
