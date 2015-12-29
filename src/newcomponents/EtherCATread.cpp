#include "EtherCATread.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATREAD;

EtherCATread::EtherCATread(const string& name) : TaskContext(name, PreOperational)
{
	//! Properties
	addProperty( "bodypart_names",     	bodypart_names     ).doc("Names of bodyparts");
	
	//! Operations
	// Add Ins
    addOperation("AddAnalogIns", &EtherCATread::AddAnalogIns, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry")
		.arg("PARTNAME","String specifying the name of the part");
    addOperation("AddDigitalIns", &EtherCATread::AddDigitalIns, this, OwnThread)
		.doc("Add one or more Digital ins")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry")
		.arg("PARTNAME","String specifying the name of the part");
    addOperation("AddEncoderIns", &EtherCATread::AddEncoderIns, this, OwnThread)
		.doc("Add one or more Encoder ins")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry")
		.arg("PARTNAME","String specifying the name of the part");
		
	// Apply (math) function
	addOperation("AddAddition_A", &EtherCATread::AddAddition_A, this, OwnThread)
		.doc("This function will add to all analog values of intput i the value as set in values[i]")
		.arg("BPID","Bodypart number of the analog in")
		.arg("PORTNR","Output port number of the analog in")
		.arg("VALUES","Doubles specifying with which the input should be added");	
	addOperation("AddMultiply_A", &EtherCATread::AddMultiply_A, this, OwnThread)
		.doc("This function will multiply all analog values of intput i with the value as set in factor[i]")
		.arg("BPID","Bodypart number of the analog in")
		.arg("PORTNR","Output port number of the analog in")
		.arg("VALUES","Doubles specifying with which the input should be multiplied");	
		
	addOperation("AddFlip_D", &EtherCATread::AddFlip_D, this, OwnThread)
		.doc("This function will flip all digital values i for which the property flip[i] contains a 1")
		.arg("BPID","Bodypart number of the digital in")
		.arg("PORTNR","Output port number of the digital in")
		.arg("VALUES","Vector of bools specifying which input should be flipped");
		
	addOperation("AddEnc2Si_E", &EtherCATread::AddEnc2Si_E, this, OwnThread)
		.doc("This function will convert raw encoder input into si values. Note that this function unlike other also adds a renull and reset port")
		.arg("BPID","Bodypart number of the encoder ins")
		.arg("PORTNR","Output port number of the encoder ins")
		.arg("ENCODER_BITS","Saturation value of the encoder. For example: 65536 for a 16 bit encoder")
		.arg("ENC2SI","Value to convert the encoder value to an SI value. Typically 2pi/(encodersteps_per_rev*gearbox)");
	addOperation("AddMatrixTransform_E", &EtherCATread::AddMatrixTransform_E, this, OwnThread)
		.doc("This function will add a matrix multiplication on the si output of the encoders. Matrix elements have to be added with properties")
		.arg("BPID","Bodypart number of the encoder ins")
		.arg("PORTNR","Output port number of the encoder ins")
		.arg("INPUTSIZE","Size of the input of the matrix transform")
		.arg("OUTPUTSIZE","Size of the output of the matrix transform");
	addOperation( "ResetEncoders", &EtherCATread::ResetEncoders, this, OwnThread )
		.doc("Reset an encoder value to a new value, usefull for homing")
		.arg("BPID","Bodypart number of the encoder ins")
		.arg("PORTNR","Output port number of the encoder ins")
		.arg("resetvalues","Values to reset the encoder to");
}

EtherCATread::~EtherCATread(){}

bool EtherCATread::configureHook()
{
	//! init	
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	goodToGO = false;
	
	n_addedbodyparts_A = 0;
	n_addedbodyparts_D = 0;
	n_addedbodyparts_E = 0;
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		n_inports_A[l] = 0;
		n_outports_A[l] = 0;
		n_inports_D[l] = 0;
		n_outports_D[l] = 0;
		n_inports_E[l] = 0;
		n_outports_E[l] = 0;
	}
		
	return true;
}

bool EtherCATread::startHook()
{
	if (bodypart_names.size() <= 0) {
		log(Error) << "EtherCATread::startHook: Could not start EtherCATread. First add a list of the bodyparts that will be added. (at least one) " << bodypart_names.size() << "!" << endlog();
		return false;
	}
	
	return true;
}

void EtherCATread::updateHook()
{
	// Functions checks the connections, (It does this once after X seconds)
	CheckAllConnections();
	
	// Reads inputs, this extracts data from ethercat slaves
	ReadInputs();
	
	// Maps the inputs into intermediate output structure (Input reshuffling, allows for example mapping X inputs on Y outputs)
	MapInputs2Outputs();
	
	// Functions to do all the calculations on the incoming data, for example converting enc counts to si values, or multiplying inputs with factor X
	Calculate_A();
	Calculate_D();
	Calculate_E();
	
	// Function to write the output calculated onto the output port
    WriteOutputs();   
    
	return;
}

void EtherCATread::AddAnalogIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	uint BPID;
		
	//! Check configuration
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Since EtherCATread component has not yet been started!" << endlog();
		return;
	}
	// Check if the AnalogIns is already added for this bodypart
	for (uint l = 0; l < MAX_BODYPARTS; l++) {
		if (PARTNAME == added_bodyparts_A[l]) {
			log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. There is already an AnalogIns for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}

	// Check for invalid number of ports
    if (N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }    
    if (N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1 || INPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Invalid inport dimension: " << INPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1 || OUTPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. Invalid outport dimension: " << OUTPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values!" << endlog();
        return;
    }
	
	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint k = 0; k < N_OUTPORT_ENTRIES; k++) {
        if( FROM_WHICH_INPORT[k] < 0 || FROM_WHICH_INPORT[k] > N_INPORTS ) {
            log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. 0 < From_which_inport <= N_INPORTS. -> 0 < " << FROM_WHICH_INPORT[k] << " <= "<< N_INPORTS <<"!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[k] < 0 || FROM_WHICH_ENTRY[k] > INPORT_DIMENSIONS[FROM_WHICH_INPORT[k]-1] ) {
            log(Error) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Could not add AnalogIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[k] << " which does not exist for inport no. " << FROM_WHICH_INPORT[k] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
	for( uint i = 0; i < N_INPORTS; i++ ) {
		addPort( PARTNAME+"_Ain"+to_string(i+1), inports_A[BPID-1][i] );
	}
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Aout"+to_string(i+1), outports_A[BPID-1][i] );
    }

    //! And the temperary properties can be added to the global properties
    n_addedbodyparts_A++;
    n_inports_A[BPID-1] = N_INPORTS;
    n_outports_A[BPID-1] = N_OUTPORTS;
    
    added_bodyparts_A[BPID-1] = PARTNAME;
    inport_dimensions_A[BPID-1].resize(N_INPORTS);
    outport_dimensions_A[BPID-1].resize(N_OUTPORTS);
    from_which_inport_A[BPID-1].resize(N_OUTPORT_ENTRIES);
    from_which_entry_A[BPID-1].resize(N_OUTPORT_ENTRIES);

    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_A[BPID-1][i] = INPORT_DIMENSIONS[i];
	}
	
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
		outport_dimensions_A[BPID-1][i] = OUTPORT_DIMENSIONS[i];
	}
	
	for( uint k = 0; k < N_OUTPORT_ENTRIES; k++ ) {
		from_which_inport_A[BPID-1][k] = FROM_WHICH_INPORT[k];
		from_which_entry_A[BPID-1][k] = FROM_WHICH_ENTRY[k];
	}
	
	// math statuses
	addition_status_A[BPID-1].resize(N_OUTPORTS);
	multiply_status_A[BPID-1].resize(N_OUTPORTS);
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		addition_status_A[BPID-1][i] = false;
		multiply_status_A[BPID-1][i] = false;
	}
	
	// Resizing math properties 
	addition_values_A[BPID-1].resize(n_outports_A[BPID-1]);
	multiply_values_A[BPID-1].resize(n_outports_A[BPID-1]);

    //! Resizing in- and outport messages
    input_msgs_A[BPID-1].resize(n_inports_A[BPID-1]);
    for( uint i = 0; i < n_inports_A[BPID-1]; i++ ) {
        input_msgs_A[BPID-1][i].values.resize( inport_dimensions_A[BPID-1][i] );
	}
		
    intermediate_A[BPID-1].resize(n_outports_A[BPID-1]);
    output_A[BPID-1].resize(n_outports_A[BPID-1]);
    for( uint i = 0; i < n_outports_A[BPID-1]; i++ ) {
        intermediate_A[BPID-1][i].resize( outport_dimensions_A[BPID-1][i]);
        output_A[BPID-1][i].resize(outport_dimensions_A[BPID-1][i]);
    }
    
    log(Warning) << "EtherCATread::AddAnalogIns(" << PARTNAME << "): Added AnalogIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	return;
}

void EtherCATread::AddDigitalIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{	
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	uint BPID;	

	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. Since EtherCATread component has not yet been started!" << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIn. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }    

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
	for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1 || INPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. Invalid inport dimension: " << INPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1 || OUTPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. Invalid outport dimension: " << OUTPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. The number of entries in from_which_inport_D and from_which_entry_D should equal the total number of output values!" << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint k = 0; k < N_OUTPORT_ENTRIES; k++) {
        if( FROM_WHICH_INPORT[k] <= 0 || FROM_WHICH_INPORT[k] > N_INPORTS ) {
            log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. 0 < From_which_inport <= N_INPORTS. -> 0 < " << FROM_WHICH_INPORT[k] << " <= "<< N_INPORTS <<"!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[k] <= 0 || FROM_WHICH_ENTRY[k] > INPORT_DIMENSIONS[FROM_WHICH_INPORT[k]-1] ) {
            log(Error) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Could not add DigitalIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[k] << " which does not exist for inport no. " << FROM_WHICH_INPORT[k] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
	for( uint i = 0; i < N_INPORTS; i++ ) {
		addPort( PARTNAME+"_Din"+to_string(i+1), inports_D[BPID-1][i] );
	}
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Dout"+to_string(i+1), outports_D[BPID-1][i] );
    }
    
    //! And the temperary properties can be added to the global properties
    n_addedbodyparts_D++;
    n_inports_D[BPID-1] = N_INPORTS;
    n_outports_D[BPID-1] = N_OUTPORTS;
        
    added_bodyparts_D[BPID-1] = PARTNAME;
    inport_dimensions_D[BPID-1].resize(N_INPORTS);
    outport_dimensions_D[BPID-1].resize(N_OUTPORTS);
    from_which_inport_D[BPID-1].resize(N_OUTPORT_ENTRIES);
    from_which_entry_D[BPID-1].resize(N_OUTPORT_ENTRIES);
    
    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_D[BPID-1][i] = INPORT_DIMENSIONS[i];
	}
	
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
		outport_dimensions_D[BPID-1][i] = OUTPORT_DIMENSIONS[i];
	}
	
	for( uint k = 0; k < N_OUTPORT_ENTRIES; k++ ) {
		from_which_inport_D[BPID-1][k] = FROM_WHICH_INPORT[k];
		from_which_entry_D[BPID-1][k] = FROM_WHICH_ENTRY[k];
	}
	
	// math statuses
	flip_status_D[BPID-1].resize(N_OUTPORTS);
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		flip_status_D[BPID-1][i] = false;
	}
	
	// Resizing math properties
	flip_values_D[BPID-1].resize(n_outports_D[BPID-1]);

    
	//! Resizing in- and outport messages
    input_msgs_D[BPID-1].resize(n_inports_D[BPID-1]);
    for( uint i = 0; i < n_inports_D[BPID-1]; i++ ) {
        input_msgs_D[BPID-1][i].values.resize( inport_dimensions_D[BPID-1][i] );
	}
		
    intermediate_D[BPID-1].resize(n_outports_D[BPID-1]);
    output_D[BPID-1].resize(n_outports_D[BPID-1]);
    for( uint i = 0; i < n_outports_D[BPID-1]; i++ ) {
        intermediate_D[BPID-1][i].resize( outport_dimensions_D[BPID-1][i]);
        output_D[BPID-1][i].resize(outport_dimensions_D[BPID-1][i]);
    }
    
    log(Warning) << "EtherCATread::AddDigitalIns(" << PARTNAME << "): Added DigitalIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	return;
}

void EtherCATread::AddEncoderIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{	
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	uint BPID;
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	// Check if the EncoderIns is already added for this bodypart
	for(uint l = 0; l < MAX_BODYPARTS; l++) {
		if (PARTNAME == added_bodyparts_E[l]) {
			log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. There is already an EncoderIns for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add Endoder in. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_ENCPORTS) {
        log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }
    
	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
	// Special case for Encoders are that all inport_dimensions should be one
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] != 1) {
            log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. Inport_dimensions cannot contain value other than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    
    if(OUTPORT_DIMENSIONS.size() != 1) {
		log(Warning) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Outport_dimensions is advised to be of size 1!" << endlog();
	}
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. The number of entries in from_which_inport_E and from_which_entry_E should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint k = 0; k < N_OUTPORT_ENTRIES; k++) {
        if( FROM_WHICH_INPORT[k] <= 0 || FROM_WHICH_INPORT[k] > N_INPORTS ) {
            log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. 0 < From_which_inport <= N_INPORTS. -> 0 < " << FROM_WHICH_INPORT[k] << " <= "<< N_INPORTS <<"!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[k] != 1 ) {
            log(Error) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Could not add EncoderIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[k] << " which should be 1 for a encoder in.!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
	for( uint i = 0; i < N_INPORTS; i++ ) {
		addPort( PARTNAME+"_Ein"+to_string(i+1), inports_E[BPID-1][i] );
	}
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Eout"+to_string(i+1), outports_E[BPID-1][i] );
        addPort( PARTNAME+"_Eout_vel"+to_string(i+1), outports_E_vel[BPID-1][i] );
    }
    
    //! And the temperary properties can be added to the global properties
    n_addedbodyparts_E++;
    n_inports_E[BPID-1] = N_INPORTS;
    n_outports_E[BPID-1] = N_OUTPORTS;
    
    added_bodyparts_E[BPID-1] = PARTNAME;
    inport_dimensions_E[BPID-1].resize(N_INPORTS);
    outport_dimensions_E[BPID-1].resize(N_OUTPORTS);
    from_which_inport_E[BPID-1].resize(N_OUTPORT_ENTRIES);
    from_which_entry_E[BPID-1].resize(N_OUTPORT_ENTRIES);
    
    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_E[BPID-1][i] = (int) INPORT_DIMENSIONS[i];
	}
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		outport_dimensions_E[BPID-1][i] = OUTPORT_DIMENSIONS[i];
	}
	for( uint k = 0; k < N_OUTPORT_ENTRIES; k++ ) {
		from_which_inport_E[BPID-1][k] = ((int) FROM_WHICH_INPORT[k]);
		from_which_entry_E[BPID-1][k] = ((int) FROM_WHICH_ENTRY[k]);
	}
	
	// math statuses
	enc2si_status_E[BPID-1].resize(N_OUTPORTS);
	matrixtransform_status_E[BPID-1].resize(N_OUTPORTS);	
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		enc2si_status_E[BPID-1][i] = false;
		matrixtransform_status_E[BPID-1][i] = false;
	}
	
	// Resizing math properties 
	enc2si_values_E[BPID-1].resize(n_outports_E[BPID-1]);
	encoderbits_E[BPID-1].resize(n_outports_E[BPID-1]);
	matrixtransform_entries_E[BPID-1].resize(n_outports_E[BPID-1]);

	initpos_E[BPID-1].resize(n_outports_E[BPID-1]);
	enc_values[BPID-1].resize(n_outports_E[BPID-1]);
	previous_enc_values[BPID-1].resize(n_outports_E[BPID-1]);
	encodercntr_E[BPID-1].resize(n_outports_E[BPID-1]);
    
    //! Resizing in- and outport messages
    input_msgs_E[BPID-1].resize(n_inports_E[BPID-1]);    
    intermediate_E[BPID-1].resize(n_outports_E[BPID-1]);
    output_E[BPID-1].resize(n_outports_E[BPID-1]);
    output_E_vel[BPID-1].resize(n_outports_E[BPID-1]);
    
    for( uint i = 0; i < n_outports_E[BPID-1]; i++ ) {
        intermediate_E[BPID-1][i].resize( outport_dimensions_E[BPID-1][i] );
        output_E[BPID-1][i].resize( outport_dimensions_E[BPID-1][i] );
        output_E_vel[BPID-1][i].resize( outport_dimensions_E[BPID-1][i] );
    }
    
    log(Warning) << "EtherCATread::AddEncoderIns(" << PARTNAME << "): Added EncoderIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	return;
}

//! Functions to edit inputs
// Analog
void EtherCATread::AddAddition_A(string PARTNAME, int PORTNR, doubles ADDVALUES)
{
	// init
	uint BPID;
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Could not add Addition_A. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread(" << PARTNAME << ")::AddAddition_A: BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread(" << PARTNAME << ")::AddAddition_A: Could not add addition. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	if( PORTNR <= 0 || PORTNR > n_outports_A[BPID-1]) {
		log(Error) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Could not add addition. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_A[BPID-1] << "!" << endlog();
		return;
	}
	if( ADDVALUES.size() != outport_dimensions_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Could not add addition. Invalid size of ADDVALUES " << ADDVALUES.size() << ". Should have been of size :" << outport_dimensions_A[BPID-1][PORTNR-1] << "!" << endlog();
		return;
	}
	if( multiply_status_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Could not add addition. For this output a multiplier is already there" << endlog();
		log(Error) << "If you want to do both a multiply and an addition, then do the addition first and then the multiply" << endlog();
		return;
	}

	// Resize and save math properties
	addition_values_A[BPID-1][PORTNR-1].resize(outport_dimensions_A[BPID-1][PORTNR-1]);
	for( uint k = 0; k < outport_dimensions_A[BPID-1][PORTNR-1]; k++ ) {
		addition_values_A[BPID-1][PORTNR-1][k] = ADDVALUES[k];
	}		
		
	if( addition_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Overwritten existing addition." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddAddition_A(" << PARTNAME << "): Added addition." << endlog();
	}
	
	// Set status
	addition_status_A[BPID-1][PORTNR-1] = true;
}

void EtherCATread::AddMultiply_A(string PARTNAME, int PORTNR, doubles MULTIPLYFACTOR)
{
	// init
	uint BPID;	
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddMultiply_A(" << PARTNAME << "): Could not add Multiply_A. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread(" << PARTNAME << ")::AddMultiply_A: BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread(" << PARTNAME << ")::AddMultiply_A: Could not add Multiply. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_outports_A[BPID-1]) {
		log(Error) << "EtherCATread::AddMultiply_A(" << PARTNAME << "): Could not add multiply. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_A[BPID-1] << "!" << endlog();
		return;
	}
	if( MULTIPLYFACTOR.size() != outport_dimensions_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATread::AddMultiply_A(" << PARTNAME << "): Could not add multiplier. Invalid size of MULTIPLYFACTOR " << MULTIPLYFACTOR.size() << ". Should have be of size :" << outport_dimensions_A[BPID-1][PORTNR-1] << "!" << endlog();
		return;
	}	
	
	// Resize and save math properties
	multiply_values_A[BPID-1][PORTNR-1].resize(outport_dimensions_A[BPID-1][PORTNR-1]);
	for( uint k = 0; k < outport_dimensions_A[BPID-1][PORTNR-1]; k++ ) {
		multiply_values_A[BPID-1][PORTNR-1][k] = MULTIPLYFACTOR[k];
	}
	
	if( multiply_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddMultiply_A(" << PARTNAME << "): Overwritten existing multiplier." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddMultiply_A(" << PARTNAME << "): Added multiplier." << endlog();
	}
	
	// Set status
	multiply_status_A[BPID-1][PORTNR-1] = true;
	
}

// Digital
void EtherCATread::AddFlip_D(string PARTNAME, int PORTNR, doubles FLIPVALUES)
{
	// init
	uint BPID;	
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddFlip_D(" << PARTNAME << "): Could not add Flip_D. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread(" << PARTNAME << ")::AddFlip_D: BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread(" << PARTNAME << ")::AddFlip_D: Could not add Flip. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_outports_D[BPID-1]) {
		log(Error) << "EtherCATread::AddFlip_D(" << PARTNAME << "): Could not add flip. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_D[BPID-1] << "!" << endlog();
		return;
	}
	if( FLIPVALUES.size() != outport_dimensions_D[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATread::AddFlip_D(" << PARTNAME << "): Could not add flip. Invalid size of FLIPVALUES. Should have be of size :" << outport_dimensions_D[BPID-1][PORTNR-1] << "!" << endlog();
		return;
	}
	
	// Resize and save math properties
	flip_values_D[BPID-1][PORTNR-1].resize(outport_dimensions_D[BPID-1][PORTNR-1]);
	for( uint k = 0; k < outport_dimensions_D[BPID-1][PORTNR-1]; k++ ) {
		flip_values_D[BPID-1][PORTNR-1][k] = FLIPVALUES[k];
	}
	
	if( flip_status_D[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddFlip_D(" << PARTNAME << "): Overwritten existing flip." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddFlip_D(" << PARTNAME << "): Added a flip." << endlog();
	}
	
	// Set status
	flip_status_D[BPID-1][PORTNR-1] = true;
}

// Encoder
void EtherCATread::AddEnc2Si_E(string PARTNAME, int PORTNR, doubles ENCODERBITS, doubles ENC2SI)
{
	// init
	uint BPID;	
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Could not add Enc2Si_E. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread(" << PARTNAME << ")::AddEnc2Si_E: BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread(" << PARTNAME << ")::AddEnc2Si_E: Could not add Enc2SI. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_outports_E[BPID-1]) {
		log(Error) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Could not add enc2si. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_E[BPID-1] << "!" << endlog();
		return;
	}
	if( ENCODERBITS.size() != outport_dimensions_E[BPID-1][PORTNR-1] || ENC2SI.size() != outport_dimensions_E[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Could not add enc2si. Invalid size of ENC2SI. Should have be of size :" << outport_dimensions_E[BPID-1][PORTNR-1] << "!" << endlog();
		return;
	}
	for(uint k = 0; k < ENCODERBITS.size(); k++) {
		if(ENC2SI[k] > 1.0 ) {
			log(Error) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Could not add enc2si. Currently Enc2SI values > 1 are not supported. Your " << k <<"th enc2si was: " << ENC2SI[k] << "!" << endlog();
			return;
		}
	}
	
	// Resize and save math properties
	encoderbits_E[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	enc2si_values_E[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	initpos_E[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	enc_values[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	previous_enc_values[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	encodercntr_E[BPID-1][PORTNR-1].resize(ENCODERBITS.size());
	
	for(uint k = 0; k < ENCODERBITS.size(); k++) {
		encoderbits_E[BPID-1][PORTNR-1][k] = ENCODERBITS[k]; 
		enc2si_values_E[BPID-1][PORTNR-1][k] = ENC2SI[k];
		
		initpos_E[BPID-1][PORTNR-1][k] = 0.0;
		enc_values[BPID-1][PORTNR-1][k] = 0.0;
		previous_enc_values[BPID-1][PORTNR-1][k] = 0.0;
		encodercntr_E[BPID-1][PORTNR-1][k] = 0;
	}

	if( enc2si_status_E[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Overwritten existing enc2si." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddEnc2Si_E(" << PARTNAME << "): Added a enc2si." << endlog();
	}	
	
	// Set status
	enc2si_status_E[BPID-1][PORTNR-1] = true;
	
	// Almost done, first convert to si for the first time
	ReadInputs();
	MapInputs2Outputs();
	Calculate_E();
	
	// Then reset Encoders
	doubles zerovec(ENCODERBITS.size(),0.0);
	ResetEncoders(BPID, PORTNR, zerovec);

}

void EtherCATread::AddMatrixTransform_E(string PARTNAME, int PORTNR, double INPUTSIZE, double OUTPUTSIZE)
{
	// Matrix transform is only supported over 1 output port.
	
	// init
	uint BPID;
		
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATread::AddMatrixTransform_E(" << PARTNAME << "): Could not add MatrixTransform_E. Since EtherCATread component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATread(" << PARTNAME << ")::AddMatrixTransform_E: BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread(" << PARTNAME << ")::AddMatrixTransform_E: Could not add MatrixTransform. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATread!" << endlog();
		return;
	}
	 
	if( PORTNR <= 0 || PORTNR > n_outports_E[BPID-1]) {
		log(Error) << "EtherCATread::AddMatrixMultiplication_E(" << PARTNAME << "): Could not add matrix transform. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_E[BPID-1] << "!" << endlog();
		return;
	}
	if( matrixtransform_status_E[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddMatrixTransform_E(" << PARTNAME << "): Could not add Matrix Transform, since this already excists for this bodypart. Overwriting is not supported at the moment" << endlog();
		return;
	}
	if( OUTPUTSIZE != outport_dimensions_E[BPID-1][PORTNR-1]) {
		if( OUTPUTSIZE < outport_dimensions_E[BPID-1][PORTNR-1]) { 	// smaller matrix then nr of inputs is allowed, not recommended
			log(Warning) << "EtherCATread::AddMatrixTransform_E(" << PARTNAME << "): INPUTSIZE: " << OUTPUTSIZE << " is smafller than the size of the outport outport_dimensions_E[BPID-1][PORTNR-1]:" << outport_dimensions_E[BPID-1][PORTNR-1] << "!" << endlog();
		} else { 										// larger matrix then nr of inputs is not allowed
			log(Error) << "EtherCATread::AddMatrixTransform_E(" << PARTNAME << "): Could not add matrix transform. INPUTSIZE: " << OUTPUTSIZE << " is larger than the size of the outport outport_dimensions_E[BPID-1][PORTNR-1]:" << inport_dimensions_E[BPID-1][PORTNR-1] << "!" << endlog();
			return;
		}
	}
	
	// Update outport_dimensions_E
	outport_dimensions_E[BPID-1][PORTNR-1] = OUTPUTSIZE;
		
	// Add property to store Matrix
	matrixtransform_entries_E[BPID-1][PORTNR-1].resize(outport_dimensions_E[BPID-1][PORTNR-1]);
	for ( uint k = 0; k < outport_dimensions_E[BPID-1][PORTNR-1]; k++ ) {
		
		matrixtransform_entries_E[BPID-1][PORTNR-1][k].resize(outport_dimensions_E[BPID-1][PORTNR-1]); 
		string name = added_bodyparts_E[BPID-1]+to_string(PORTNR)+"_matrixtransform"+to_string(k+1);
		addProperty( name, matrixtransform_entries_E[BPID-1][PORTNR-1][k]);

		// Set matrix default to Identity matrix
		for ( uint j = 0; j < matrixtransform_entries_E[BPID-1][PORTNR-1][k].size(); j++ ) { 
			matrixtransform_entries_E[BPID-1][PORTNR-1][k][j] = 0.0;
		}
		matrixtransform_entries_E[BPID-1][PORTNR-1][k][k] = 1.0;
	}
	
	if( !matrixtransform_status_E[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATread::AddMatrixTransform_E(" << PARTNAME << "): Added a matrix transform." << endlog();
	}
	
	// Set status
	matrixtransform_status_E[BPID-1][PORTNR-1] = true;
}

//! Functions to edit inputs
void EtherCATread::CheckAllConnections()
{
	//! 8 seconds after boot, Check all connections
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 8.0)) {
		goodToGO = true;
		
		// AnalogIns
		for(uint l = 0; l < MAX_BODYPARTS; l++) {		
			for(uint i = 0; i < n_inports_A[l]; i++) {
				if ( !inports_A[l][i].connected() ) {
					log(Error) << "EtherCATread::CheckAllConnections: Analog inport " << inports_A[l][i].getName() << " is not connected!" << endlog();
				}
			}
			for(uint i = 0; i < n_outports_A[l]; i++) {
				if ( !outports_A[l][i].connected() ) {
					log(Info) << "EtherCATread::CheckAllConnections: Analog outport " << outports_A[l][i].getName() << " is not connected!" << endlog();
				}
			}
		}
		
		// DigitalIns
		for(uint l = 0; l < MAX_BODYPARTS; l++) {		
			for(uint i = 0; i < n_inports_D[l]; i++) {
				if ( !inports_D[l][i].connected() ) {
					log(Error) << "EtherCATread::CheckAllConnections: Digital inport " << inports_D[l][i].getName() << " is not connected!" << endlog();
				}
			}
			for(uint i = 0; i < n_outports_D[l]; i++) {
				if ( !outports_D[l][i].connected() ) {
					log(Info) << "EtherCATread::CheckAllConnections: Digital outport " << outports_D[l][i].getName() << " is not connected!" << endlog();
				}
			}
		}
		
		// EncoderIns
		for(uint l = 0; l < MAX_BODYPARTS; l++) {	
			for(uint i = 0; i < n_inports_E[l]; i++) {
				if ( !inports_E[l][i].connected() ) {
					log(Error) << "EtherCATread::CheckAllConnections: Encoder inport " << inports_E[l][i].getName() << " is not connected!" << endlog();
				}
			}
			for(uint i = 0; i < n_outports_E[l]; i++) {
				if ( !outports_E[l][i].connected() ) {
					log(Info) << "EtherCATread::CheckAllConnections: Encoder outport " << outports_E[l][i].getName() << " is not connected!" << endlog();
				}
				if ( !outports_E_vel[l][i].connected() ) {
					log(Info) << "EtherCATread::CheckAllConnections: Encoder velocity outport " << outports_E_vel[l][i].getName() << " is not connected!" << endlog();
				}
			}		
		}
	}
	
}

void EtherCATread::ReadInputs()
{
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_A[l]; i++ ) {
			inports_A[l][i].read(input_msgs_A[l][i]);
		}    
	}
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_D[l]; i++ ) {
			inports_D[l][i].read(input_msgs_D[l][i]);
		}    
	}
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_E[l]; i++ ) {
			inports_E[l][i].read(input_msgs_E[l][i]);
		}
	}
}

void EtherCATread::WriteOutputs()
{
	// Analog
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			outports_A[l][i].write(output_A[l][i]);
		}
	}
	
	// Digital
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_D[l]; i++ ) {
			outports_D[l][i].write(output_D[l][i]);
		}
	}
	
	// Encoder
    for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_E[l]; i++ ) {
			outports_E[l][i].write(output_E[l][i]);
			outports_E_vel[l][i].write(output_E_vel[l][i]);
		}
	}
}

void EtherCATread::MapInputs2Outputs()
{
	// Analog
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		uint j = 0;
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			for( uint k = 0; k < outport_dimensions_A[l][i]; ++k) {
				intermediate_A[l][i][k] = input_msgs_A[l][from_which_inport_A[l][j+k]-1 ].values[from_which_entry_A[l][j+k]-1];
			}
			j += outport_dimensions_A[l][i];
		}
	}

	// Digital
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		uint j = 0;
		for( uint i = 0; i < n_outports_D[l]; i++ ) {
			for( uint k = 0; k < outport_dimensions_D[l][i]; ++k) {
				intermediate_D[l][i][k] = input_msgs_D[l][from_which_inport_D[l][j+k]-1 ].values[from_which_entry_D[l][j+k]-1];
			}
			j += outport_dimensions_D[l][i];
		}
	}
	
	// Encoder
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		uint j = 0;
		for( uint i = 0; i < n_outports_E[l]; i++ ) {
			for( uint k = 0; k < outport_dimensions_E[l][i]; ++k) {
				//log(Warning) << "EtherCATread::MapInputs2Outputs: intermediate_E[l"<< l << "][i"<< i << "][k"<< k << "][j+k"<< j+k << "] = (double) input_msgs_E[l"<< l << "][ " << from_which_inport_E[l][j+k]-1 << "], " << input_msgs_E[l][from_which_inport_E[l][j+k]-1].value << "!" << endlog();
				intermediate_E[l][i][k] = (double) input_msgs_E[l][ from_which_inport_E[l][j+k]-1 ].value;
			}
			j += outport_dimensions_E[l][i];
		}
	}
}

void EtherCATread::Calculate_A()
{
	// Output = intermediate
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		output_A[l] = intermediate_A[l];
	}
	
	// Addition
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			if (addition_status_A[l][i]) {
				for ( uint k = 0; k < outport_dimensions_A[l][i]; k++ ) {
					output_A[l][i][k] = output_A[l][i][k]+addition_values_A[l][i][k];
				}
			}
		}
	}
	
	// Multiplier
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			if (multiply_status_A[l][i]) {
				for ( uint k = 0; k < outport_dimensions_A[l][i]; k++ ) {
					output_A[l][i][k] = output_A[l][i][k]*multiply_values_A[l][i][k];
				}
			}
		}
	}
}

void EtherCATread::Calculate_D()
{
	// Output = intermediate
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		output_D[l] = intermediate_D[l];
	}
	
	// Flip
    for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_D[l]; i++ ) {
			if (flip_status_D[l][i]) {
				for ( uint k = 0; k < outport_dimensions_D[l][i]; k++ ) {
					if (flip_values_D[l][i][k]) {
						output_D[l][i][k] = !output_D[l][i][k];
					}
				}
			}
		}
	}
}

void EtherCATread::Calculate_E()
{
	// Output = intermediate
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_E[l]; i++ ) {
			output_E[l][i] = intermediate_E[l][i];
		}
	}
	
	// Enc2Si
	double dt = determineDt();
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {	
		for( uint i = 0; i < n_outports_E[l]; i++ ) {
			if (enc2si_status_E[l][i]) {
				for( uint k = 0; k < outport_dimensions_E[l][i]; ++k) {
					
					// Recent encoder value
					enc_values[l][i][k] = output_E[l][i][k];
				
					// Detect if and count the amount of times going through the maximum encoder bits
					if( (previous_enc_values[l][i][k] - enc_values[l][i][k]) > encoderbits_E[l][i][k]/2) {
						encodercntr_E[l][i][k]++;
					} else if( (previous_enc_values[l][i][k]  - enc_values[l][i][k]) < (-1 * encoderbits_E[l][i][k]/2) ) {
						encodercntr_E[l][i][k]--;
					}
					previous_enc_values[l][i][k] = output_E[l][i][k];
					 
					// Calculate SI  nr of cycles * enc bits * enc2si_values_E         + enc_values      * enc2si_values_E      - Init_SI_value (for homing)
					output_E[l][i][k] = encodercntr_E[l][i][k]*(encoderbits_E[l][i][k]*enc2si_values_E[l][i][k]) + enc_values[l][i][k]*enc2si_values_E[l][i][k] - initpos_E[l][i][k];
					
					// Calculate velocity
					output_E_vel[l][i][k] = ( ( (double) (enc_values[l][i][k]- previous_enc_values[l][i][k]) ) * enc2si_values_E[l][i][k])/dt;
					
				}
			}
		}
	}
	
	// Matrix Transform
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_E[l]; i++ ) {
			if (matrixtransform_status_E[l][i]) {
				
				// Store output of enc2si operation into a temporary input vector
				doubles input_MT_E = output_E[l][i];
				doubles input_MT_E_vel = output_E_vel[l][i];
				
				// Resize output vectors in case of a non square Matrix transformation
				output_E[l][i].resize(outport_dimensions_E[l][i]);
				output_E_vel[l][i].resize(outport_dimensions_E[l][i]);
				
				for ( uint k = 0; k < outport_dimensions_E[l][i]; k++ ) {
					output_E[l][i][k] = 0.0;
					output_E_vel[l][i][k] = 0.0;
					
					for ( uint m = 0; m < input_MT_E.size(); m++ ) {
						
						output_E[l][i][k] += matrixtransform_entries_E[l][i][k][m] * input_MT_E[m];
						output_E_vel[l][i][k] += matrixtransform_entries_E[l][i][k][m] * input_MT_E_vel[m];
					
					}
				}
			}
		}
	}
}

double EtherCATread::determineDt()
{
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    double dt = (new_time - old_time);
    old_time = new_time;
    return dt;
}

void EtherCATread::ResetEncoders(int BPID, int PORTNR, doubles RESETVALUES )
{
	if( BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATread::ResetEncoders(" << BPID << bodypart_names[BPID-1] << ", port " << PORTNR << "): Could not reset encoders. Invalid BPID: " << BPID << ".  1 <= BPID <= " << bodypart_names.size() << "!" << endlog();
		return;
	}
	if( PORTNR <= 0 || PORTNR > n_outports_E[BPID-1]) {
		log(Error) << "EtherCATread::ResetEncoders(" << BPID << bodypart_names[BPID-1] << ", port " << PORTNR << "): Could not reset encoders. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_outports_E[BPID-1] << "!" << endlog();
		return;
	}
	if (!enc2si_status_E[BPID-1][PORTNR-1]) {
		log(Error)<<"EtherCATread::ResetEncoders(" << BPID << bodypart_names[BPID-1] << ", port " << PORTNR << "): Could not reset encoders. The function AddEnc2Si has not yet been called for bodypart with BPID : " << BPID <<"."<<endlog();
		return;		
	}
	if (RESETVALUES.size() != outport_dimensions_E[BPID-1][PORTNR-1]) {
		log(Error)<<"EtherCATread::ResetEncoders(" << BPID << bodypart_names[BPID-1] << ", port " << PORTNR << "): Could not reset encoders. Received incorrect sized initialize signal: " << RESETVALUES.size() << ". Should be of size" << outport_dimensions_E[BPID-1][PORTNR-1] <<"."<<endlog();
		return;
	}

	for ( uint k = 0; k < RESETVALUES.size(); k++ ) {
		
		// Reset encodercntr_E, p
		encodercntr_E[BPID-1][PORTNR-1][k] = 0;
		
		// Set initpos_E and previous_enc_values
		initpos_E[BPID-1][PORTNR-1][k] = output_E[BPID-1][PORTNR-1][k] - RESETVALUES[k];
		previous_enc_values[BPID-1][PORTNR-1][k] = initpos_E[BPID-1][PORTNR-1][k];
		
	}
}

ORO_CREATE_COMPONENT(ETHERCATREAD::EtherCATread)
