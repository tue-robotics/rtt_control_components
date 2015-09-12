#include "EtherCATread.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATREAD;

EtherCATread::EtherCATread(const string& name) : TaskContext(name, PreOperational)
{
	//! Operations
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
		
	//! Modify existing Analog/Digital/Encoder Ins
	// Analog
	addOperation("AddAddition_A", &EtherCATread::AddAddition_A, this, OwnThread)
		.doc("This function will add to all analog values of intput i the value as set in values[i]")
		.arg("id","ID number of the digital in")
		.arg("values","Doubles specifying with which the input should be added");	
	addOperation("AddMultiply_A", &EtherCATread::AddMultiply_A, this, OwnThread)
		.doc("This function will multiply all analog values of intput i with the value as set in factor[i]")
		.arg("id","ID number of the digital in")
		.arg("factor","Doubles specifying with which the input should be multiplied");	
	// Digital
	addOperation("AddFlip_D", &EtherCATread::AddFlip_D, this, OwnThread)
		.doc("This function will flip all digital values i for which the property flip[i] contains a 1")
		.arg("id","ID number of the digital in")
		.arg("flip","Vector of bools specifying which input should be flipped");
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
	}
}

bool EtherCATread::startHook(){}

void EtherCATread::updateHook()
{
	//! 8 seconds after boot, Check all connections
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 4.0)) {
		goodToGO = true;
		CheckAllConnections();
	}

	ReadInputs();
	
	MapInput2Outputs();
	
	Calculate_A();
	Calculate_D();
	Calculate_E();
	
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
	
	log(Warning) << "EtherCATread (" << PARTNAME << ")::ADDING: [ " << INPORT_DIMENSIONS.size() << " , " << OUTPORT_DIMENSIONS.size() << " , " << FROM_WHICH_INPORT.size() << " , " << FROM_WHICH_ENTRY.size() << " ]!" << endlog();

	//! Check configuration
	// Check if the AnalogIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_A.size(); l++) {
		if (PARTNAME == added_bodyparts_A[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_A+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_A!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1-n_inports_A] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Could not add AnalogIn. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
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
    for( uint i = 0; i < n_outports_A; i++ ) {
        output_A[i].resize( outport_dimensions_A[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Succesfully added AnalogIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddAnalogIns: Total inports are now " << n_inports_A << " inports and " << n_outports_A << " outports!" << endlog();

	return;
}

void EtherCATread::AddDigitalIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	log(Warning) << "EtherCATread (" << PARTNAME << ")::ADDING: [ " << INPORT_DIMENSIONS.size() << " , " << OUTPORT_DIMENSIONS.size() << " , " << FROM_WHICH_INPORT.size() << " , " << FROM_WHICH_ENTRY.size() << " ]!" << endlog();

	//! Check configuration	
	// Check if the DigitalIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_D.size(); l++) {
		if (PARTNAME == added_bodyparts_D[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add AnalogIn. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. The number of entries in from_which_inport_D and from_which_entry_D should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_D+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_D!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1-n_inports_D] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Could not add DigitalIn. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
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
    for( uint i = 0; i < n_outports_D; i++ ) {
        output_D[i].resize( outport_dimensions_D[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Succesfully added DigitalIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddDigitalIns: Total inports are now " << n_inports_D << " inports and " << n_outports_D << " outports!" << endlog();

	return;
}

void EtherCATread::AddEncoderIns(doubles ENCODERBITS, doubles ENC2SI string PARTNAME)
{	
	//! Checks
	for(uint l = 0; l < added_bodyparts_E.size(); l++) {
		if (PARTNAME == added_bodyparts_E[l]) {
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. There is already an EncoderIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
	if(ENCODERBITS.size() != ENC2SI.size()) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. Size of ENCODERBITS and ENC2SI should match [" << ENCODERBITS.size() << " != " <<  ENC2SI.size() << "]!" << endlog();
        return;
    }
	if(n_inports_E + ENCODERBITS.size() > MAX_ENCPORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. The number of encoder inports will exceed maximum: " << n_inports_E + N_INPORTS << " > " <<  MAX_ENCPORTS << "!" << endlog();
        return;
    }
    
    //! Init
    n_outports_E++;
    uint N_INPORTS = ENCODERBITS.size();

    //! Now that all inputs have been properly examined. The in- and outports can be created
	for( uint i = 0; i < N_INPORTS; i++ ) {
		addEventPort( PARTNAME+"_Ein"+to_string(i+1), inports_E[i+n_inports_E] );
	}
	addPort( PARTNAME+"_Eout", outports_E[n_outports_E-1] );
    
    //! And the temperary properties can be added to the global properties
    added_bodyparts_E.push_back(PARTNAME);
    n_inports_E += N_INPORTS;
    encoderbits_E[n_outports_E-1].resize(N_INPORTS);
    enc2si[n_outports_E-1].resize(N_INPORTS);
    
	for( uint i = 0; i < N_INPORTS; i++ ) {
		encoderbits_E[n_outports_E-1][i] = (int) ENCODERBITS[i];
		enc2si[n_outports_E-1][i] = ENC2SI[i];
	}

    //! Resizing in- and outport messages
    input_msgs_E.resize(n_inports_E);
    output_E.resize(n_outports_E);
    output_E[n_outports_E-1].resize(N_INPORTS);
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Succesfully added EncoderIns with " << N_INPORTS << " inports and 1 outport!" << endlog();
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Total inports are now " << n_inports_E << " inports and " << n_outports_E << " outports!" << endlog();
    
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
		log(Warning) << "EtherCATread::AddAddition_A: Overwritten existing addition!" << endlog();
	} else {
		log(Warning) << "EtherCATread::AddAddition_A: Succesfully added addition!" << endlog();
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
		log(Warning) << "EtherCATread::AddMultiply_A: Overwritten existing multiplier!" << endlog();
	} else {
		log(Warning) << "EtherCATread::AddMultiply_A: Succesfully added multiplier!" << endlog();
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
		log(Warning) << "EtherCATread::AddMultiply_A: Overwritten existing multiplier!" << endlog();
	} else {
		log(Warning) << "EtherCATread::AddFlip_D: Succesfully added a flip!" << endlog();
	}
	
	// Set status
	flip_status_D[ID-1] = true;
}

//! Functions to edit inputs
void EtherCATread::CheckAllConnections()
{
	// AnalogIns
	for(uint i = 0; i < n_inports_A; i++) {
		if ( !inports_A[i].connected() ) {
			log(Error) << "EtherCATread::CheckAllConnections: Analog inport " << inports_A[i].getName() << " is not connected." << endlog();
		}
	}
	for(uint i = 0; i < n_outports_A; i++) {
		if ( !outports_A[i].connected() ) {
			log(Info) << "EtherCATread::CheckAllConnections: Analog outport " << outports_A[i].getName() << " is not connected." << endlog();
		}
	}
	
	// DigitalIns
	for(uint i = 0; i < n_inports_D; i++) {
		if ( !inports_D[i].connected() ) {
			log(Error) << "EtherCATread::CheckAllConnections: Digital inport " << inports_D[i].getName() << " is not connected." << endlog();
		}
	}
	for(uint i = 0; i < n_outports_D; i++) {
		if ( !outports_D[i].connected() ) {
			log(Info) << "EtherCATread::CheckAllConnections: Digital outport " << outports_D[i].getName() << " is not connected." << endlog();
		}
	}
	
	// EncoderIns
	for(uint i = 0; i < n_inports_E; i++) {
		if ( !inports_E[i].connected() ) {
			log(Error) << "EtherCATread::CheckAllConnections: Encoder inport " << inports_E[i].getName() << " is not connected." << endlog();
		}
	}
	for(uint i = 0; i < n_outports_E; i++) {
		if ( !outports_E[i].connected() ) {
			log(Info) << "EtherCATread::CheckAllConnections: Encoder outport " << outports_E[i].getName() << " is not connected." << endlog();
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
	}	
}

void EtherCATread::MapInput2Outputs()
{	
    uint j = 0;
    for( uint i = 0; i < n_outports_A; i++ ) {
        for( uint k = 0; k < outport_dimensions_A[i]; ++k) {
            output_A[i][k] = input_msgs_A[ from_which_inport_A[j]-1 ].values[ from_which_entry_A[j]-1 ];
            j++;   
        }
    }

    j = 0;
    for( uint i = 0; i < n_outports_D; i++ ) {
        for( uint k = 0; k < outport_dimensions_D[i]; ++k) {
            output_D[i][k] = input_msgs_D[ from_which_inport_D[j]-1 ].values[ from_which_entry_D[j]-1 ];
            j++;
        }
    }
    
}

void EtherCATread::Calculate_A()
{	
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
	// Calculate dt
    double dt = determineDt();
	
	// i loops over all inports
	// l loops over the bodyparts
	// k loops over the inputs of this bodypart
 	
	uint l = 0;
	uint k =0;
	for ( uint i = 0; i < n_inports_E; i++ ) {			// i loops over inports
		if (i == encoderbits_E[j].size()) {
			l++;
			k = 0;
		}
		
        SI_values[i] = readEncoder(i,j,k);
        enc_velocity[i] = ((double)(enc_position[i]-enc_position_prev[i])*enc2SI[i])/dt;
        enc_position_prev[i] = enc_position[i];
        
        k++;
    }

    outport.write(SI_values);
    outport_vel.write(enc_velocity);
    outport_enc.write(ENC_values);
    counter++;
	
	
	
	
	
	
	// First convert encoder counts 2 SI
    j = 0;
    for( uint i = 0; i < n_outports_E; i++ ) {
        for( uint k = 0; k < output_E.size(); ++k) {
            output_E[i][k] = input_msgs_E[j].value;
            j++;
        }   
    }
}

double EtherCATread::readEncoder(int i, int j, int k )
{
    EncoderMsg encdata;
    inport_enc[i].read(encdata);

    uint new_enc_position = encdata.value;
    ENC_values[i] = encdata.value;
    if( (previous_enc_position[i] - new_enc_position) > encoderbits/2)
        ienc[i]++;
    else if( (previous_enc_position[i] - new_enc_position) < (-1 * (double)encoderbits/2))
        ienc[i]--;
    previous_enc_position[i] = new_enc_position;
    enc_position[i] = ienc[i] * encoderbits + new_enc_position;
    double SI_value =  ((double)enc_position[i] * enc2SI[i]) - init_SI_values[i] + offsets[i];
    return SI_value;
}

double EtherCATread::determineDt()
{
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    double dt = (new_time - old_time);
    old_time = new_time;
    return dt;
}

ORO_CREATE_COMPONENT(ETHERCATREAD::EtherCATread)
