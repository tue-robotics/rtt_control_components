#include "EtherCATwrite.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATWRITE;

EtherCATwrite::EtherCATwrite(const string& name) : TaskContext(name, PreOperational)
{
	//! Properties
	addProperty( "bodypart_names",     	bodypart_names     ).doc("Names of bodyparts");	
	
	//! Operations
    addOperation("AddAnalogOuts", &EtherCATwrite::AddAnalogOuts, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("PARTNAME","String specifying the name of the part")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry");
    addOperation("AddDigitalOuts", &EtherCATwrite::AddDigitalOuts, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("PARTNAME","String specifying the name of the part")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry");
	
	//! Add math
	// Analog
	addOperation("AddAddition_A", &EtherCATwrite::AddAddition_A, this, OwnThread)
		.doc("This function will add to all analog values of intput i the value as set in values[i]")
		.arg("BPID","Bodypart number of the analog out")
		.arg("PORTNR","Output port number of the analog out")
		.arg("values","Doubles specifying with which the input should be added");	
	addOperation("AddMultiply_A", &EtherCATwrite::AddMultiply_A, this, OwnThread)
		.doc("This function will multiply all analog values of intput i with the value as set in factor[i]")
		.arg("BPID","Bodypart number of the analog out")
		.arg("PORTNR","Output port number of the analog out")
		.arg("factor","Doubles specifying with which the input should be multiplied");	
	addOperation("AddMatrixTransform_A", &EtherCATwrite::AddMatrixTransform_A, this, OwnThread)
		.doc("This function will add a matrix multiplication on the analog output. Matrix defaults to I, but should be updated using properties")
		.arg("BPID","Bodypart number of the analog out")
		.arg("PORTNR","Output port number of the analog out")
		.arg("INPUTSIZE","Size of the input of the matrix transform")
		.arg("OUTPUTSIZE","Size of the output of the matrix transform");
}
EtherCATwrite::~EtherCATwrite(){}

bool EtherCATwrite::configureHook()
{
	//! init
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	goodToGO = false;
	
	n_addedbodyparts_A = 0;
	n_addedbodyparts_D = 0;
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		n_inports_A[l] = 0;
		n_outports_A[l] = 0;
		n_inports_D[l] = 0;
		n_outports_D[l] = 0;
	}
	
	return true;
}

bool EtherCATwrite::startHook()
{
	if (bodypart_names.size() <= 0) {
		log(Error) << "EtherCATwrite::startHook: Could not start EtherCATwrite. First add a list of the bodyparts that will be added. (at least one) " << bodypart_names.size() << "!" << endlog();
		return false;
	}
	
	return true;
}

void EtherCATwrite::updateHook()
{
	// Functions checks the connections, (It does this once after X seconds)
	CheckAllConnections();
	
	// Reads inputs, this extracts data from ethercat slaves
	ReadInputs();
		
	// Functions to do all the calculations on the incoming data, for example converting enc counts to si values, or multiplying inputs with factor X
	Calculate_A();
	Calculate_D();

	// Maps the inputs into output structure (Input reshuffling, allows for example mapping X inputs on Y outputs)
	MapInputs2Outputs();
	
	// Function to write the output calculated onto the output port
    WriteOutputs();

	return;
}

void EtherCATwrite::AddAnalogOuts(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	uint BPID;
	
	//! Check configuration
	if (!this->isRunning()) {
		log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Since EtherCATwrite component has not yet been started!" << endlog();
		return;
	}
	
	// Check if the AnalogOuts is already added for this bodypart
	for(uint l = 0; l < MAX_BODYPARTS; l++) {
		if (PARTNAME == added_bodyparts_A[l]) {
			log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
		
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATwrite!" << endlog();
		return;
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint k = 0; k < N_OUTPORT_ENTRIES; k++) {
        if( FROM_WHICH_INPORT[k] < 0 || FROM_WHICH_INPORT[k] > N_INPORTS ) {
            log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. 0 < From_which_inport <= N_INPORTS. -> 0 < " << FROM_WHICH_INPORT[k] << " <= "<< N_INPORTS <<"!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[k] < 0 || FROM_WHICH_ENTRY[k] > INPORT_DIMENSIONS[FROM_WHICH_INPORT[k]-1] ) {
            log(Error) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Could not add AnalogOuts. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[k] << " which does not exist for inport no. " << FROM_WHICH_INPORT[k] << "!" << endlog();
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
	addition_status_A[BPID-1].resize(N_INPORTS);
	multiply_status_A[BPID-1].resize(N_INPORTS);
	matrixtransform_status_A[BPID-1].resize(N_INPORTS);
    for( uint i = 0; i < N_INPORTS; i++ ) {
		addition_status_A[BPID-1][i] = false;
		multiply_status_A[BPID-1][i] = false;
		matrixtransform_status_A[BPID-1][i] = false;
	}
	
	// Resizing math properties 
	addition_values_A[BPID-1].resize(n_inports_A[BPID-1]);
	multiply_values_A[BPID-1].resize(n_inports_A[BPID-1]);
	matrixtransform_entries_A[BPID-1].resize(n_inports_A[BPID-1]);		    
	    
    //! Resizing in- and outport messages
    input_A[BPID-1].resize(n_inports_A[BPID-1]);
    for( uint i = 0; i < n_inports_A[BPID-1]; i++ ) {
        input_A[BPID-1][i].resize( inport_dimensions_A[BPID-1][i] );
	}

    output_msgs_A[BPID-1].resize(n_outports_A[BPID-1]);
    for( uint i = 0; i < n_outports_A[BPID-1]; i++ ) {
        output_msgs_A[BPID-1][i].values.resize( outport_dimensions_A[BPID-1][i] );
    }
    
    log(Warning) << "EtherCATwrite::AddAnalogOuts(" << PARTNAME << "): Added AnalogOuts with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    
	return;
}

void EtherCATwrite::AddDigitalOuts(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	uint BPID;
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Since EtherCATwrite component has not yet been started!" << endlog();
		return;
	}
	// Check if the DigitalOuts is already added for this bodypart
	for(uint l = 0; l < MAX_BODYPARTS; l++) {
		if (PARTNAME == added_bodyparts_D[l]) {
			log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. There is already an DigitalOuts for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATwrite!" << endlog();
		return;
	}
	    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. The number of entries in from_which_inport_D and from_which_entry_D should equal the total number of output values." << endlog();
        return;
    }

    for(uint k = 0; k < N_OUTPORT_ENTRIES; k++) {
        if( FROM_WHICH_INPORT[k] <= 0 || FROM_WHICH_INPORT[k] > N_INPORTS ) {
            log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. 0 < From_which_inport <= N_INPORTS. -> 0 < " << FROM_WHICH_INPORT[k] << " <= "<< N_INPORTS <<"!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[k] <= 0 || FROM_WHICH_ENTRY[k] > INPORT_DIMENSIONS[FROM_WHICH_INPORT[k]-1] ) {
            log(Error) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Could not add DigitalOuts. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[k] << " which does not exist for inport no. " << FROM_WHICH_INPORT[k] << "!" << endlog();
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
	
    //! Resizing in- and outport messages
    input_D[BPID-1].resize(n_inports_D[BPID-1]);
    for( uint i = 0; i < n_inports_D[BPID-1]; i++ ) {
        input_D[BPID-1][i].resize( inport_dimensions_D[BPID-1][i] );
	}

    output_msgs_D[BPID-1].resize(n_outports_D[BPID-1]);
    for( uint i = 0; i < n_outports_D[BPID-1]; i++ ) {
        output_msgs_D[BPID-1][i].values.resize(outport_dimensions_D[BPID-1][i]);
    }
    
    log(Warning) << "EtherCATwrite::AddDigitalOuts(" << PARTNAME << "): Succesfully added DigitalOuts with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();

	return;
}

//! Functions to edit inputs
// Analog
void EtherCATwrite::AddAddition_A(string PARTNAME, int PORTNR, doubles ADDVALUES)
{
	// init
	uint BPID;	
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATwrite::AddAddition_A(" << PARTNAME << "): Could not add AddAddition_A. Since EtherCATwrite component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATwrite::AddMultiply_A(" << PORTNR << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATwrite::AddMultiply_A(" << PORTNR << "): Could not add DigitalOuts. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATwrite!" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_inports_A[BPID-1]) {
		log(Error) << "EtherCATwrite::AddAddition_A(" << PORTNR << "): Could not add addition. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_inports_A[BPID-1] << "!" << endlog();
		return;
	}
	if( ADDVALUES.size() != inport_dimensions_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATwrite::AddAddition_A(" << PARTNAME << "): Could not add addition. Invalid size of ADDVALUES " << ADDVALUES.size() << ". Should have been of size :" << inport_dimensions_A[BPID-1][PORTNR-1] << "!" << endlog();
		return;
	}
	if( addition_status_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATwrite::AddAddition_A: Could not add addition. For this input a multiplier is already there" << endlog();
		log(Error) << "If you want to do both a multiply and an addition, then do the addition first and then the mulitiply" << endlog();
		return;
	}
	
	// Save math properties
	addition_values_A[BPID-1][PORTNR-1].resize(inport_dimensions_A[BPID-1][PORTNR-1]);
	for( uint k = 0; k < inport_dimensions_A[BPID-1][PORTNR-1]; k++ ) {
		addition_values_A[BPID-1][PORTNR-1][k] = ADDVALUES[k];
	}
	
	if( addition_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATwrite::AddAddition_A(" << PARTNAME << "): Overwritten existing addition." << endlog();
	} else {
		log(Warning) << "EtherCATwrite::AddAddition_A(" << PARTNAME << "): Added addition." << endlog();
	}
	
	// Set status
	addition_status_A[BPID-1][PORTNR-1] = true;
}

void EtherCATwrite::AddMultiply_A(string PARTNAME, int PORTNR, doubles MULTIPLYFACTOR)
{
	// init
	uint BPID;	
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATwrite::AddMultiply_A(" << PARTNAME << "): Could not add Multiply_A. Since EtherCATwrite component has not yet been started." << endlog();
		return;
	}
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATwrite::AddMultiply_A(" << PORTNR << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATwrite::AddMultiply_A(" << PORTNR << "): Could not add DigitalOuts. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATwrite!" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_inports_A[BPID-1]) {
		log(Error) << "EtherCATwrite::AddMultiply_A(" << PORTNR << "): Could not add multiply. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_inports_A[BPID-1] << "!" << endlog();
		return;
	}
	if( MULTIPLYFACTOR.size() != inport_dimensions_A[BPID-1][PORTNR-1] ) {
		log(Error) << "EtherCATwrite::AddMultiply_A: Could not add multiplier. Invalid size of MULTIPLYFACTOR. Should have be of size :" << MULTIPLYFACTOR.size() << "!" << endlog();
		return;
	}
	
	// Save math properties
	multiply_values_A[BPID-1][PORTNR-1].resize(inport_dimensions_A[BPID-1][PORTNR-1]);
	for( uint k = 0; k < inport_dimensions_A[BPID-1][PORTNR-1]; k++ ) {
		multiply_values_A[BPID-1][PORTNR-1][k] = MULTIPLYFACTOR[k];
	}
	
	if( multiply_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATwrite::AddMultiply_A(" << PARTNAME << "): Overwritten existing multiplier." << endlog();
	} else {
		log(Warning) << "EtherCATwrite::AddMultiply_A(" << PARTNAME << "): Added multiplier." << endlog();
	}
	
	// Set status
	multiply_status_A[BPID-1][PORTNR-1] = true;
	
}

void EtherCATwrite::AddMatrixTransform_A(string PARTNAME, int PORTNR, double INPUTSIZE, double OUTPUTSIZE)
{
	// init
	uint BPID;
	
	//! Check configuration	
	if (!this->isRunning()) {
		log(Error) << "EtherCATwrite::AddMatrixTransform_A(" << PARTNAME << "): Could not add MatrixTransform_A. Since EtherCATwrite component has not yet been started." << endlog();
		return;
	}
	
	// Check if the bodypart name is in the bodypart_name list and set the BPID accordingly
	for (uint l = 0; l < bodypart_names.size(); l++) {
		if (bodypart_names[l] == PARTNAME) {
			BPID = l + 1;
			log(Info) << "EtherCATwrite::AddMatrixTransform_A(" << PORTNR << "): BPID IS: " << BPID << "." << endlog();
		}
	}
	if (BPID <= 0 || BPID > bodypart_names.size()) {
		log(Error) << "EtherCATwrite::AddMatrixTransform_A(" << PORTNR << "): Could not add DigitalOuts. Could not find " << PARTNAME << " in the list of bodypartnames of EtherCATwrite!" << endlog();
		return;
	}
	if( matrixtransform_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATwrite::AddMatrixTransform_A: Could not add Matrix Transform, since this already excists for this bodypart. Overwriting is not supported at the moment" << endlog();
		return;
	}
	
	if( PORTNR <= 0 || PORTNR > n_inports_A[BPID-1]) {
		log(Error) << "EtherCATwrite::AddMatrixTransform_A(" << PORTNR << "): Could not add matrix transform. Invalid PORTNR: " << PORTNR << ".  1 <= PORTNR <= " << n_inports_A[BPID-1] << "!" << endlog();
		return;
	}
	
	if( INPUTSIZE != inport_dimensions_A[BPID-1][PORTNR-1]) { 
		if( INPUTSIZE < inport_dimensions_A[BPID-1][PORTNR-1]) { 	// smaller matrix then nr of inputs is allowed, not recommended
			log(Warning) << "EtherCATwrite::AddMatrixTransform_A: INPUTSIZE: " << INPUTSIZE << " is smaller than the size of the inport inport_dimensions_A[BPID-1]:" << inport_dimensions_A[BPID-1][PORTNR-1] << "!" << endlog();
		} else { 										// larger matrix then nr of inputs is not allowed
			log(Error) << "EtherCATwrite::AddMatrixTransform_A: Could not add matrix transform. INPUTSIZE: " << INPUTSIZE << " is larger than the size of the inport inport_dimensions_A[BPID-1]:" << inport_dimensions_A[BPID-1][PORTNR-1] << "!" << endlog();
			return;
		}
	}
	
	// Add property to store Matrix
	matrixtransform_entries_A[BPID-1][PORTNR-1].resize(inport_dimensions_A[BPID-1][PORTNR-1]);
	for ( uint k = 0; k < inport_dimensions_A[BPID-1][PORTNR-1]; k++ ) {
		
		matrixtransform_entries_A[BPID-1][PORTNR-1][k].resize(inport_dimensions_A[BPID-1][PORTNR-1]); 
		string name = added_bodyparts_A[BPID-1]+to_string(PORTNR)+"_matrixtransform"+to_string(k+1);
		addProperty( name, matrixtransform_entries_A[BPID-1][PORTNR-1][k]);
		
		// Set matrix default to Identity matrix
		for ( uint j = 0; j < matrixtransform_entries_A[BPID-1][PORTNR-1][k].size(); j++ ) { 
			matrixtransform_entries_A[BPID-1][PORTNR-1][k][j] = 0.0;
		}
		matrixtransform_entries_A[BPID-1][PORTNR-1][k][k] = 1.0;
	}
	
	if( !matrixtransform_status_A[BPID-1][PORTNR-1] ) {
		log(Warning) << "EtherCATwrite::AddMatrixTransform_A(" << PARTNAME << "): Added a matrix transform." << endlog();
	}
	
	// Set status
	matrixtransform_status_A[BPID-1][PORTNR-1] = true;
	
}

void EtherCATwrite::CheckAllConnections()
{
	//! 8 seconds after boot, Check all connections
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 8.0)) {
		goodToGO = true;
	
		// AnalogOuts
		for(uint l = 0; l < MAX_BODYPARTS; l++) {
			for(uint i = 0; i < n_inports_A[l]; i++) {
				if ( !inports_A[l][i].connected() ) {
					log(Error) << "EtherCATwrite::CheckAllConnections: Analog inport " << inports_A[l][i].getName() << " is not connected!" << endlog();
				}
			}
			for(uint i = 0; i < n_outports_A[l]; i++) {
				if ( !outports_A[l][i].connected() ) {
					log(Info) << "EtherCATwrite::CheckAllConnections: Analog outport " << outports_A[l][i].getName() << " is not connected!" << endlog();
				}
			}
		}
		
		// DigitalOuts
		for(uint l = 0; l < MAX_BODYPARTS; l++) {
			for(uint i = 0; i < n_inports_D[l]; i++) {
				if ( !inports_D[l][i].connected() ) {
					log(Error) << "EtherCATwrite::CheckAllConnections: Digital inport " << inports_D[l][i].getName() << " is not connected!" << endlog();
				}
			}
			for(uint i = 0; i < n_outports_D[l]; i++) {
				if ( !outports_D[l][i].connected() ) {
					log(Info) << "EtherCATwrite::CheckAllConnections: Digital outport " << outports_D[l][i].getName() << " is not connected!" << endlog();
				}
			}
		}		
	}	
}

void EtherCATwrite::Calculate_A()
{	
	// Addition
    for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_A[l]; i++ ) {
			if (addition_status_A[l][i]) {
				for( uint k = 0; k < inport_dimensions_A[l][i]; ++k) {
					input_A[l][i][k] += addition_values_A[l][i][k];
				}
			}
		}
	}
	
	// Multiplier
    for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_A[l]; i++ ) {
			if (multiply_status_A[l][i]) {
				for( uint k = 0; k < inport_dimensions_A[l][i]; ++k) {
					input_A[l][i][k] = input_A[l][i][k]*multiply_values_A[l][i][k];
				}
			}
		}
	}
	
	// Matrix Transform	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_A[l]; i++ ) {
			if (matrixtransform_status_A[l][i]) {
				
				// Store input_A into a temporary input vector
				doubles input_MT_A = input_A[l][i];
				
				// Resize output vectors in case of a non square Matrix transformation
				input_A[l][i].resize(inport_dimensions_A[l][i]);
				
				for ( uint k = 0; k < inport_dimensions_A[l][i]; k++ ) {
					input_A[l][i][k] = 0.0;
					
					for ( uint j = 0; j < input_MT_A.size(); j++ ) {
						input_A[l][i][k] += matrixtransform_entries_A[l][i][k][j] * input_MT_A[j];
					}
				}
			}
		}
	}	
}

void EtherCATwrite::Calculate_D()
{
	// No D math operations are supported at the moment
}

void EtherCATwrite::ReadInputs()
{
	// analog
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {	
		for( uint i = 0; i < n_inports_A[l]; i++ ) {
			inports_A[l][i].read(input_A[l][i]);
		}
	}
	
	// digital
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_inports_D[l]; i++ ) {
			inports_D[l][i].read(input_D[l][i]);
		}
	}
}

void EtherCATwrite::WriteOutputs()
{	
	// analog
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			outports_A[l][i].write(output_msgs_A[l][i]);
		}
	}

	// digital
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		for( uint i = 0; i < n_outports_D[l]; i++ ) {
			outports_D[l][i].write(output_msgs_D[l][i]);
		}
	}
}

void EtherCATwrite::MapInputs2Outputs()
{
	// Do mapping of input entries on into output
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		uint j = 0;
		for( uint i = 0; i < n_outports_A[l]; i++ ) {
			for( uint k = 0; k < outport_dimensions_A[l][i]; ++k) {
				if (from_which_inport_A[l][j+k] == 0 || from_which_entry_A[l][j+k] == 0) {
					output_msgs_A[l][i].values[k] = 0.0;
				} else {
					output_msgs_A[l][i].values[k] = input_A[l][ from_which_inport_A[l][j+k]-1 ][ from_which_entry_A[l][j+k]-1 ];
				}
			}
			j += outport_dimensions_A[l][i];
		}
	}

	// Do mapping of input entries on into output
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		uint j = 0;
		for( uint i = 0; i < n_outports_D[l]; i++ ) {
			for( uint k = 0; k < outport_dimensions_D[l][i]; ++k) {
				if (from_which_inport_D[l][j+k] == 0 || from_which_entry_D[l][j+k] ==0) {
					output_msgs_D[l][i].values[k] = 0;
				} else {
					output_msgs_D[l][i].values[k] = input_D[l][ from_which_inport_D[l][j+k]-1 ][ from_which_entry_D[l][j+k]-1 ];
				}
			}
			j += outport_dimensions_D[l][i];
		}
	}
}

ORO_CREATE_COMPONENT(ETHERCATWRITE::EtherCATwrite)
