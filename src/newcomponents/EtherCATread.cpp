#include "EtherCATread.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATREAD;

EtherCATread::EtherCATread(const string& name) : TaskContext(name, PreOperational)
{
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
		.arg("ID","ID number of the digital in")
		.arg("VALUES","Doubles specifying with which the input should be added");	
	addOperation("AddMultiply_A", &EtherCATread::AddMultiply_A, this, OwnThread)
		.doc("This function will multiply all analog values of intput i with the value as set in factor[i]")
		.arg("ID","ID number of the digital in")
		.arg("FACTOR","Doubles specifying with which the input should be multiplied");	
		
	addOperation("AddFlip_D", &EtherCATread::AddFlip_D, this, OwnThread)
		.doc("This function will flip all digital values i for which the property flip[i] contains a 1")
		.arg("ID","ID number of the digital in")
		.arg("FLIP","Vector of bools specifying which input should be flipped");
		
	addOperation("AddEnc2Si_E", &EtherCATread::AddEnc2Si_E, this, OwnThread)
		.doc("This function will convert raw encoder input into si values. Note that this function unlike other also adds a renull and reset port")
		.arg("ID","ID number of the encoder ins")
		.arg("ENCODER_BITS","Saturation value of the encoder. For example: 65536 for a 16 bit encoder")
		.arg("ENC2SI","Value to convert the encoder value to an SI value. Typically 2pi/(encodersteps_per_rev*gearbox)");
	addOperation("AddMatrixTransform_E", &EtherCATread::AddMatrixTransform_E, this, OwnThread)
		.doc("This function will add a matrix multiplication on the si output of the encoders. Only input is ID. Matrix should be added with properties")
		.arg("ID","ID number of the encoder ins")
		.arg("INPUTSIZE","Size of the input of the matrix transform")
		.arg("OUTPUTSIZE","Size of the output of the matrix transform");
	addOperation( "ResetEncoders", &EtherCATread::ResetEncoders, this, OwnThread )
		.doc("Reset an encoder value to a new value, usefull for homing")
		.arg("ID","ID number of the encoder ins")
		.arg("resetvalues","Values to reset the encoder to");
}

EtherCATread::~EtherCATread(){}

bool EtherCATread::configureHook()
{
	//! init	
	// Global
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	goodToGO = false;
	
	// Analog
	n_inports_A = 0;
	n_outports_A = 0;
	n_inport_entries_A = 0;
	n_outport_entries_A = 0;

	// Digital
	n_inports_D = 0;
	n_outports_D = 0;
	n_inport_entries_D = 0;
	n_outport_entries_D = 0;
	
	// Encoder
	n_inports_E = 0;
	n_outports_E = 0;
	n_inport_entries_E = 0;
	n_outport_entries_E = 0;
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		addition_status_A[l] = false;
		multiply_status_A[l] = false;
		flip_status_D[l] = false;
		enc2si_status_E[l] = false;
		matrixtransform_status_E[l] = false;
	}
	
	return true;
}

bool EtherCATread::startHook(){}

void EtherCATread::updateHook()
{
	// Functions checks the connections, (It does this once after X seconds)
	CheckAllConnections();
	
	// Reads inputs, this extracts data from ethercat slaves
	ReadInputs();
	
	// Maps the inputs into intermediate output structure (Input reshuffling, allows for example mapping X inputs on Y outputs)
	MapInput2Outputs();		
	
	// Functions to do all the calculations on the incoming data, for example converting enc counts to si values, or multiplying inputs with factor X
	Calculate_A();			
	Calculate_D();
	Calculate_E();
	
	// Function to write the output calculated onto the output port
    WriteOutputs();
    
	return;
}

void EtherCATread::AddAnalogIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	//! Check configuration
	// Check if the AnalogIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_A.size(); l++) {
		if (PARTNAME == added_bodyparts_A[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. There is already an AnalogIns for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1 || INPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. Invalid inport dimension: " << INPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1 || OUTPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. Invalid outport dimension: " << OUTPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_A+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_A!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1-n_inports_A] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
    for( uint i = n_inports_A; i < (n_inports_A+N_INPORTS); i++ ) {
		addEventPort( PARTNAME+"_Ain"+to_string(i+1-n_inports_A), inports_A[i] );
	}
    for( uint i = n_outports_A; i < (n_outports_A+N_OUTPORTS); i++ ) {
        addPort( PARTNAME+"_Aout"+to_string(i+1-n_outports_A), outports_A[i] );
    }
    
    //! Update from_which_inport property (Necessary since when in the ops file the first inport is selected this means the first inport for this bodypart)
    for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		if (n_inports_A != 0) {
			FROM_WHICH_INPORT[j] += (double) n_inports_A;
		}
	}
    
    //! And the temperary properties can be added to the global properties
    added_bodyparts_A.push_back(PARTNAME);
    n_inports_A += N_INPORTS;
    n_outports_A += N_OUTPORTS;
    n_inport_entries_A += N_INPORT_ENTRIES;
    n_outport_entries_A += N_OUTPORT_ENTRIES;
    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_A.push_back((int) INPORT_DIMENSIONS[i]);
	}
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		outport_dimensions_A.push_back((int) OUTPORT_DIMENSIONS[i]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_inport_A.push_back((int) FROM_WHICH_INPORT[j]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_entry_A.push_back((int) FROM_WHICH_ENTRY[j]);
	}
    
    //! Resizing in- and outport messages
    input_msgs_A.resize(n_inports_A);
    for( uint i = 0; i < n_inports_A; i++ ) {
        input_msgs_A[i].values.resize( inport_dimensions_A[i] );
	}

    output_A.resize(n_outports_A);
    intermediate_A.resize(n_outports_A);
    for( uint i = 0; i < n_outports_A; i++ ) {
        output_A[i].resize( outport_dimensions_A[i] );
        intermediate_A[i].resize( outport_dimensions_A[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Added AnalogIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	return;
}

void EtherCATread::AddDigitalIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	//! Check configuration	
	// Check if the DigitalIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_D.size(); l++) {
		if (PARTNAME == added_bodyparts_D[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. There is already an DigitalIns for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
	for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1 || INPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. Invalid inport dimension: " << INPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1 || OUTPORT_DIMENSIONS[i] > 100) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. Invalid outport dimension: " << OUTPORT_DIMENSIONS[i] << "!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. The number of entries in from_which_inport_D and from_which_entry_D should equal the total number of output values!" << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_D+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_D!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1-n_inports_D] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
    for( uint i = n_inports_D; i < (n_inports_D+N_INPORTS); i++ ) {
		addEventPort( PARTNAME+"_Din"+to_string(i+1-n_inports_D), inports_D[i] );
	}
    for( uint i = n_outports_D; i < (n_outports_D+N_OUTPORTS); i++ ) {
        addPort( PARTNAME+"_Dout"+to_string(i+1-n_outports_D), outports_D[i] );
    }
    
    //! Update from_which_inport property (Necessary since when in the ops file the first inport is selected this means the first inport for this bodypart)
    for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		if (n_inports_D != 0) {
			FROM_WHICH_INPORT[j] += (double) n_inports_D;
		}
	}
    
    //! And the temperary properties can be added to the global properties
    added_bodyparts_D.push_back(PARTNAME);
    n_inports_D += N_INPORTS;
    n_outports_D += N_OUTPORTS;
    n_inport_entries_D += N_INPORT_ENTRIES;
    n_outport_entries_D += N_OUTPORT_ENTRIES;
    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_D.push_back((int) INPORT_DIMENSIONS[i]);
	}
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		outport_dimensions_D.push_back((int) OUTPORT_DIMENSIONS[i]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_inport_D.push_back((int) FROM_WHICH_INPORT[j]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_entry_D.push_back((int) FROM_WHICH_ENTRY[j]);
	}
	
    //! Resizing in- and outport messages
    input_msgs_D.resize(n_inports_D);
    for( uint i = 0; i < n_inports_D; i++ ) {
        input_msgs_D[i].values.resize( inport_dimensions_D[i] );
	}

    output_D.resize(n_outports_D);
    intermediate_D.resize(n_outports_D);
    for( uint i = 0; i < n_outports_D; i++ ) {
        output_D[i].resize( outport_dimensions_D[i] );
        intermediate_D[i].resize( outport_dimensions_D[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Added DigitalIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	return;
}

void EtherCATread::AddEncoderIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{	
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	//! Check configuration	
	// Check if the EncoderIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_E.size(); l++) {
		if (PARTNAME == added_bodyparts_E[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. There is already an EncoderIns for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
	// Special case for Encoders are that all inport_dimensions should be one and there can only be one outport for each bodypart
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] != 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. Inport_dimensions cannot contain value other than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    
    if(OUTPORT_DIMENSIONS.size() != 1) {
		log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. Outport_dimensions should be of size 1. There can only be one outputport per bodypart!" << endlog();
		return;
	}
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. The number of entries in from_which_inport_E and from_which_entry_E should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_E+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_E!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIns. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
    for( uint i = n_inports_E; i < (n_inports_E+N_INPORTS); i++ ) {
		addEventPort( PARTNAME+"_Ein"+to_string(i+1-n_inports_E), inports_E[i] );
	}
    for( uint i = n_outports_E; i < (n_outports_E+N_OUTPORTS); i++ ) {
        addPort( PARTNAME+"_Eout"+to_string(i+1-n_outports_E), outports_E[i] );
        addPort( PARTNAME+"_Eout_vel"+to_string(i+1-n_outports_E), outports_E_vel[i] );
    }
    
    //! Update from_which_inport property (Necessary since when in the ops file the first inport is selected this means the first inport for this bodypart)
    for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		if (n_inports_E != 0) {
			FROM_WHICH_INPORT[j] += (double) n_inports_E;
		}
	}
    
    //! And the temperary properties can be added to the global properties
    added_bodyparts_E.push_back(PARTNAME);
    n_inports_E += N_INPORTS;
    n_outports_E += N_OUTPORTS;
    n_inport_entries_E += N_INPORT_ENTRIES;
    n_outport_entries_E += N_OUTPORT_ENTRIES;
    for( uint i = 0; i < N_INPORTS; i++ ) {
		inport_dimensions_E.push_back((int) INPORT_DIMENSIONS[i]);
	}
	for( uint i = 0; i < N_OUTPORTS; i++ ) {
		intermediate_dimensions_E.push_back((int) OUTPORT_DIMENSIONS[i]); // if a non square matrix transform is added the intermediate dimensions will remain while the outport_dimensions are updated
		outport_dimensions_E.push_back((int) OUTPORT_DIMENSIONS[i]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_inport_E.push_back((int) FROM_WHICH_INPORT[j]);
	}
	for( uint j = 0; j < N_OUTPORT_ENTRIES; j++ ) {
		from_which_entry_E.push_back((int) FROM_WHICH_ENTRY[j]);
	}
	
    //! Resizing in- and outport messages
    input_msgs_E.resize(n_inports_E);

    output_E.resize(n_outports_E);
    intermediate_E.resize(n_outports_E);
    output_E_vel.resize(n_outports_E);
    for( uint i = 0; i < n_outports_E; i++ ) {
        output_E[i].resize( outport_dimensions_E[i] );
        intermediate_E[i].resize( outport_dimensions_E[i] );
        output_E_vel[i].resize( outport_dimensions_E[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Added EncoderIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports." << endlog();

	// Do a reset encoders

	return;
}

//! Functions to edit inputs
// Analog
void EtherCATread::AddAddition_A(int ID, doubles VALUES)
{
	// Check
	if( ID <= 0 || ID > n_outports_A) {
		log(Error) << "EtherCATread::AddAddition_A: Could not add addition. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_A << "!" << endlog();
		return;
	}
	if( VALUES.size() != output_A[ID-1].size() ) {
		log(Error) << "EtherCATread::AddAddition_A: Could not add addition. Invalid size of VALUES. Should have be of size :" << output_A[ID-1].size() << "!" << endlog();
		return;
	}
	if( multiply_status_A[ID-1] ) {
		log(Error) << "EtherCATread::AddAddition_A: Could not add addition. For this output a multiplier is already there" << endlog();
		log(Error) << "If you want to do both a multiply and an addition, then do the addition first and then the mulitiply" << endlog();
		return;
	}
	
	// Resize
	addition_values_A[ID-1].resize(output_A[ID-1].size());
	
	// Save ramp properties
	for( uint i = 0; i < output_A[ID-1].size(); i++ ) {
		addition_values_A[ID-1][i] = VALUES[i];
	}
	
	if( addition_status_A[ID-1] ) {
		log(Warning) << "EtherCATread::AddAddition_A: Overwritten existing addition." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddAddition_A: Added addition." << endlog();
	}
	
	// Set status
	addition_status_A[ID-1] = true;
}

void EtherCATread::AddMultiply_A(int ID, doubles FACTOR)
{
	// Check
	if( ID <= 0 || ID > n_outports_A) {
		log(Error) << "EtherCATread::AddMultiply_A: Could not add multiplier. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_A << "!" << endlog();
		return;
	}
	if( FACTOR.size() != output_A[ID-1].size() ) {
		log(Error) << "EtherCATread::AddMultiply_A: Could not add multiplier. Invalid size of FACTOR. Should have be of size :" << output_A[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Resize
	multiply_factor_A[ID-1].resize(output_A[ID-1].size());
	
	// Save ramp properties
	for( uint i = 0; i < output_A[ID-1].size(); i++ ) {
		multiply_factor_A[ID-1][i] = FACTOR[i];
	}
	
	if( addition_status_A[ID-1] ) {
		log(Warning) << "EtherCATread::AddMultiply_A: Overwritten existing multiplier." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddMultiply_A: Added multiplier." << endlog();
	}
	
	// Set status
	multiply_status_A[ID-1] = true;
	
}

// Digital
void EtherCATread::AddFlip_D(int ID, doubles FLIP)
{
	// Check
	if( ID <= 0 || ID > n_outports_D) {
		log(Error) << "EtherCATread::AddFlip_D: Could not add flip. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_D << "!" << endlog();
		return;
	}
	if( FLIP.size() != output_D[ID-1].size() ) {
		log(Error) << "EtherCATread::AddFlip_D: Could not add flip. Invalid size of FLIP. Should have be of size :" << output_D[ID-1].size() << "!" << endlog();
		return;
	}
	
	// Resize
	flip_flip_D[ID-1].resize(output_D[ID-1].size());
	
	// Save ramp properties
	for( uint i = 0; i < output_D[ID-1].size(); i++ ) {
		flip_flip_D[ID-1][i] = (bool) FLIP[i];
	}
	
	if( flip_status_D[ID-1] ) {
		log(Warning) << "EtherCATread::AddFlip_D: Overwritten existing flip." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddFlip_D: Added a flip." << endlog();
	}
	
	// Set status
	flip_status_D[ID-1] = true;
}

// Encoder
void EtherCATread::AddEnc2Si_E(int ID, doubles ENCODERBITS, doubles ENC2SI)
{
	// Check
	if( ID <= 0 || ID > n_outports_E) {
		log(Error) << "EtherCATread::AddEnc2Si_E: Could not add enc2si. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_E << "!" << endlog();
		return;
	}
	if( ENCODERBITS.size() != output_E[ID-1].size() || ENC2SI.size() != output_E[ID-1].size() ) {
		log(Error) << "EtherCATread::AddEnc2Si_E: Could not add enc2si. Invalid size of FLIP. Should have be of size :" << output_E[ID-1].size() << "!" << endlog();
		return;
	}
	for(uint k = 0; k < ENCODERBITS.size(); k++) {
		if(ENC2SI[k] > 1.0 ) {
			log(Error) << "EtherCATread::AddEnc2Si_E: Could not add enc2si. Currently Enc2SI values > 1 are not supported. Your " << k <<"th enc2si was: " << ENC2SI[k] << "!" << endlog();
			return;
		}
	}
	
	// Store Properties
	encoderbits[ID-1].resize(ENCODERBITS.size());
	enc2SI[ID-1].resize(ENCODERBITS.size());
	position_SI_init[ID-1].resize(ENCODERBITS.size());
	enc_values[ID-1].resize(ENCODERBITS.size());
	previous_enc_values[ID-1].resize(ENCODERBITS.size());
	ienc[ID-1].resize(ENCODERBITS.size());
	
	for(uint k = 0; k < ENCODERBITS.size(); k++) {
		encoderbits[ID-1][k] = ENCODERBITS[k]; 
		enc2SI[ID-1][k] = ENC2SI[k];
		
		position_SI_init[ID-1][k] = 0.0;
		enc_values[ID-1][k] = 0.0;
		previous_enc_values[ID-1][k] = 0.0;
		ienc[ID-1][k] = 0;
	}

	if( enc2si_status_E[ID-1] ) {
		log(Warning) << "EtherCATread::AddEnc2Si_E: Overwritten existing enc2si." << endlog();
	} else {
		log(Warning) << "EtherCATread::AddEnc2Si_E: Added a enc2si." << endlog();
	}	
	
	// Set status
	enc2si_status_E[ID-1] = true;
	
	// Almost done, first convert to si for the first time
	ReadInputs();
	MapInput2Outputs();
	Calculate_E();
	
	// Then reset Encoders
	doubles zeros(ENCODERBITS.size(),0.0);
	ResetEncoders(ID, zeros);

}

void EtherCATread::AddMatrixTransform_E(int ID, double INPUTSIZE, double OUTPUTSIZE)
{
	// Check
	if( matrixtransform_status_E[ID-1] ) {
		log(Warning) << "EtherCATread::AddMatrixTransform_E: Could not add Matrix Transform, since this already excists for this bodypart. Overwriting is not supported at the moment" << endlog();
		return;
	}
	if( ID <= 0 || ID > n_outports_E) {
		log(Error) << "EtherCATread::AddMatrixMultiplication_E: Could not add matrix transform. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_E << "!" << endlog();
		return;
	}
	if( INPUTSIZE != outport_dimensions_E[ID-1]) {
		log(Error) << "EtherCATread::AddMatrixMultiplication_E: Could not add matrix transform. OUTPUTSIZE: " << OUTPUTSIZE << " != outport_dimensions_E[ID-1]" << outport_dimensions_E[ID-1] << "!" << endlog();
		return;
	}
	
	// Update outport_dimensions_E
	outport_dimensions_E[ID-1] = OUTPUTSIZE;
		
	// Resize input structs
	input_MT_E.resize(n_outports_E);
    input_MT_E_vel.resize(n_outports_E);
    for( uint i = 0; i < n_outports_E; i++ ) {
        input_MT_E[i].resize( outport_dimensions_E[ID-1] );
        input_MT_E_vel[i].resize( outport_dimensions_E[ID-1]);
    }
		
	// Add property to store Matrix
	matrixtransform_E[ID-1].resize(outport_dimensions_E[ID-1]);
	for ( uint i = 0; i < outport_dimensions_E[ID-1]; i++ ) {
		
		matrixtransform_E[ID-1][i].resize(outport_dimensions_E[ID-1]); 
		string name = added_bodyparts_E[ID-1]+"_matrixtransform"+to_string(i+1);
		addProperty( name, matrixtransform_E[ID-1][i]);
		
		// Set matrix default to Identity matrix
		for ( uint l = 0; l < matrixtransform_E[ID-1][i].size(); l++ ) { 
			matrixtransform_E[ID-1][i][l] = 0.0;
		}
		matrixtransform_E[ID-1][i][i] = 1.0;
	}
	
	if( !matrixtransform_status_E[ID-1] ) {
		log(Warning) << "EtherCATread::AddMatrixTransform_E: Added a matrix transform." << endlog();
	}
	
	// Set status
	matrixtransform_status_E[ID-1] = true;
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
		for(uint i = 0; i < n_inports_A; i++) {
			if ( !inports_A[i].connected() ) {
				log(Error) << "EtherCATread::CheckAllConnections: Analog inport " << inports_A[i].getName() << " is not connected!" << endlog();
			}
		}
		for(uint i = 0; i < n_outports_A; i++) {
			if ( !outports_A[i].connected() ) {
				log(Info) << "EtherCATread::CheckAllConnections: Analog outport " << outports_A[i].getName() << " is not connected!" << endlog();
			}
		}
		
		// DigitalIns
		for(uint i = 0; i < n_inports_D; i++) {
			if ( !inports_D[i].connected() ) {
				log(Error) << "EtherCATread::CheckAllConnections: Digital inport " << inports_D[i].getName() << " is not connected!" << endlog();
			}
		}
		for(uint i = 0; i < n_outports_D; i++) {
			if ( !outports_D[i].connected() ) {
				log(Info) << "EtherCATread::CheckAllConnections: Digital outport " << outports_D[i].getName() << " is not connected!" << endlog();
			}
		}
		
		// EncoderIns
		for(uint i = 0; i < n_inports_E; i++) {
			if ( !inports_E[i].connected() ) {
				log(Error) << "EtherCATread::CheckAllConnections: Encoder inport " << inports_E[i].getName() << " is not connected!" << endlog();
			}
		}
		for(uint i = 0; i < n_outports_E; i++) {
			if ( !outports_E[i].connected() ) {
				log(Info) << "EtherCATread::CheckAllConnections: Encoder outport " << outports_E[i].getName() << " is not connected!" << endlog();
			}
			if ( !outports_E_vel[i].connected() ) {
				log(Info) << "EtherCATread::CheckAllConnections: Encoder velocity outport " << outports_E_vel[i].getName() << " is not connected!" << endlog();
			}
		}
			
	}
}

void EtherCATread::ReadInputs()
{
	for( uint i = 0; i < n_inports_A; i++ ) {
		inports_A[i].read(input_msgs_A[i]);
    }    
    for( uint i = 0; i < n_inports_D; i++ ) {
		inports_D[i].read(input_msgs_D[i]);
    }    
    for( uint i = 0; i < n_inports_E; i++ ) {
		inports_E[i].read(input_msgs_E[i]);
    }
}

void EtherCATread::WriteOutputs()
{
    for( uint i = 0; i < n_outports_A; i++ ) {
		outports_A[i].write(output_A[i]);
	}
        
    for( uint i = 0; i < n_outports_D; i++ ) {
		outports_D[i].write(output_D[i]);
	}
	    
    for( uint i = 0; i < n_outports_E; i++ ) {
		outports_E[i].write(output_E[i]);
		outports_E_vel[i].write(output_E_vel[i]);
	}	
}

void EtherCATread::MapInput2Outputs()
{	
	// Analog
    uint j = 0;
    for( uint i = 0; i < n_outports_A; i++ ) {
        for( uint k = 0; k < outport_dimensions_A[i]; ++k) {
            intermediate_A[i][k] = input_msgs_A[ from_which_inport_A[j]-1 ].values[ from_which_entry_A[j]-1 ];
            j++;   
        }
    }

	// Digital
    j = 0;
    for( uint i = 0; i < n_outports_D; i++ ) {
        for( uint k = 0; k < outport_dimensions_D[i]; ++k) {
            intermediate_D[i][k] = input_msgs_D[ from_which_inport_D[j]-1 ].values[ from_which_entry_D[j]-1 ];
            j++;
        }
    }
    
    // Encoders
    j = 0;
    for( uint i = 0; i < n_outports_E; i++ ) {
        for( uint k = 0; k < outport_dimensions_E[i]; ++k) {
            intermediate_E[i][k] = (double) input_msgs_E[ from_which_inport_E[j]-1 ].value;
            j++;
        }
    }    
    
}

void EtherCATread::Calculate_A()
{	
	// Output = intermediate
	output_A = intermediate_A;
	
	// Addition
    for( uint i = 0; i < n_outports_A; i++ ) {
		if (addition_status_A[i]) {
			for( uint k = 0; k < output_A[i].size(); ++k) {
				output_A[i][k] = output_A[i][k]+addition_values_A[i][k];
			}
		}
    }
    
	// Multiplier
    for( uint i = 0; i < n_outports_A; i++ ) {
		if (multiply_status_A[i]) {
			for( uint k = 0; k < output_A[i].size(); ++k) {
				output_A[i][k] = output_A[i][k]*multiply_factor_A[i][k];
			}
		}
    }
}

void EtherCATread::Calculate_D()
{
	// Output = intermediate
	output_D = intermediate_D;
	
	// Flip
    for( uint i = 0; i < n_outports_D; i++ ) {
		if (flip_status_D[i]) {			
			for( uint k = 0; k < output_D[i].size(); ++k) {		
				if (flip_flip_D[i][k]) {
					output_D[i][k] = !output_D[i][k];
				}
			}
		}
    }

}

void EtherCATread::Calculate_E()
{
	// Output = intermediate
	output_E = intermediate_E;
		
	// Enc2Si
    double dt = determineDt();
	
    for( uint i = 0; i < n_outports_E; i++ ) {
		if (enc2si_status_E[i]) {
			for( uint k = 0; k < output_E[i].size(); ++k) {
				
				// Recent encoder value
				enc_values[i][k] = output_E[i][k];				
			
				// Detect if and count the amount of times going through the maximum encoder bits
				if( (previous_enc_values[i][k] - enc_values[i][k]) > encoderbits[i][k]/2) {
					ienc[i][k]++;
				} else if( (previous_enc_values[i][k]  - enc_values[i][k]) < (-1 * encoderbits[i][k]/2) ) {
					ienc[i][k]--;
				}
				previous_enc_values[i][k] = output_E[i][k];
				 
				// Calculate SI  nr of cycles * enc bits * enc2SI         + enc_values      * enc2SI      - Init_SI_value (for homing)
				output_E[i][k] = ienc[i][k]*(encoderbits[i][k]*enc2SI[i][k]) + enc_values[i][k]*enc2SI[i][k] - position_SI_init[i][k];
				
				// Calculate velocity
				output_E_vel[i][k] = ( ( (double) (enc_values[i][k]- previous_enc_values[i][k]) ) * enc2SI[i][k])/dt;
				
			}
		}
	}
	
	// Matrix Transform
    for( uint i = 0; i < n_outports_E; i++ ) {
		if (matrixtransform_status_E[i]) {
									
			input_MT_E = output_E;
			input_MT_E_vel = output_E_vel;	
						
			for ( uint k = 0; k < output_E[i].size(); k++ ) {
				output_E[i][k] = 0.0;
				output_E_vel[i][k] = 0.0;
				
				for ( uint l = 0; l < output_E[i].size(); l++ ) {
					output_E[i][k] += matrixtransform_E[i][k][l] * input_MT_E[i][l];
					output_E_vel[i][k] += matrixtransform_E[i][k][l] * input_MT_E_vel[i][l];
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

void EtherCATread::ResetEncoders(int ID, doubles resetvalues )
{
	if (ID > n_outport_entries_E) {
		log(Error)<<"ReadEncoders::ResetEncoders: Could not reset encoders. Invalid ID provided. Should lie between [0," << n_outport_entries_E <<"]"<<endlog();
		return;
	}
	if (!enc2si_status_E[ID-1]) {
		log(Error)<<"ReadEncoders::ResetEncoders: Could not reset encoders. The function AddEnc2Si has not yet been called for bodypart with ID : " << ID <<"."<<endlog();
		return;		
	}
	if (resetvalues.size() != encoderbits[ID-1].size()) {
		log(Error)<<"ReadEncoders::ResetEncoders: Could not reset encoders. Received incorrect sized initialize signal: " << resetvalues.size() << ". Should be of size" << n_inport_entries_E <<"."<<endlog();
		return;
	}

	
	for ( uint k = 0; k < resetvalues.size(); k++ ) {
		
		// Reset ienc, p
		ienc[ID-1][k] = 0;
		
		// Set position_SI_init and previous_enc_values
		position_SI_init[ID-1][k] = output_E[ID-1][k] - resetvalues[k];
		previous_enc_values[ID-1][k] = position_SI_init[ID-1][k];
		
	}
}

ORO_CREATE_COMPONENT(ETHERCATREAD::EtherCATread)
