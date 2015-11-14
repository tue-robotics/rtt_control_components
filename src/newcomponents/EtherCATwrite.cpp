#include "EtherCATwrite.hpp"

using namespace std;
using namespace RTT;
using namespace ETHERCATWRITE;

EtherCATwrite::EtherCATwrite(const string& name) : TaskContext(name, PreOperational)
{
	//! Operations
    addOperation("AddAnalogOuts", &EtherCATwrite::AddAnalogOuts, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry")
		.arg("PARTNAME","String specifying the name of the part");
    addOperation("AddDigitalOuts", &EtherCATwrite::AddDigitalOuts, this, OwnThread)
		.doc("Add one or more analog ins")
		.arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
		.arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
		.arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
		.arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry")
		.arg("PARTNAME","String specifying the name of the part");
	
	//! Add math
	// Analog
	addOperation("AddAddition_A", &EtherCATwrite::AddAddition_A, this, OwnThread)
		.doc("This function will add to all analog values of intput i the value as set in values[i]")
		.arg("id","ID number of the digital in")
		.arg("values","Doubles specifying with which the input should be added");	
	addOperation("AddMultiply_A", &EtherCATwrite::AddMultiply_A, this, OwnThread)
		.doc("This function will multiply all analog values of intput i with the value as set in factor[i]")
		.arg("id","ID number of the digital in")
		.arg("factor","Doubles specifying with which the input should be multiplied");	
	addOperation("AddMatrixTransform_A", &EtherCATwrite::AddMatrixTransform_A, this, OwnThread)
		.doc("This function will add a matrix multiplication on the analog output. Only input is ID. Matrix defaults to I, but should be updated using properties")
		.arg("ID","ID number of the encoder ins")	
		.arg("INPUTSIZE","Size of the input of the matrix transform")
		.arg("OUTPUTSIZE","Size of the output of the matrix transform");
}
EtherCATwrite::~EtherCATwrite(){}

bool EtherCATwrite::configureHook()
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
	
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		addition_status_A[l] = false;
		multiply_status_A[l] = false;
		matrixtransform_status_A[l] = false;
	}
	
	return true;
}

bool EtherCATwrite::startHook(){}

void EtherCATwrite::updateHook()
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
	
	// Function to write the output calculated onto the output port
    WriteOutputs();

	return;
}

void EtherCATwrite::AddAnalogOuts(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	log(Info) << "EtherCATwrite (" << PARTNAME << ")::ADDING: [ " << INPORT_DIMENSIONS.size() << " , " << OUTPORT_DIMENSIONS.size() << " , " << FROM_WHICH_INPORT.size() << " , " << FROM_WHICH_ENTRY.size() << " ]!" << endlog();

	//! Check configuration
	// Check if the AnalogIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_A.size(); l++) {
		if (PARTNAME == added_bodyparts_A[l]) {
			log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_A+N_INPORTS) || FROM_WHICH_INPORT[j] < 0 ) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_A!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1] || FROM_WHICH_ENTRY[j] < 0 ) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Could not add AnalogOut. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
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
    input_A.resize(n_inports_A);
    for( uint i = 0; i < n_inports_A; i++ ) {
        input_A[i].resize( inport_dimensions_A[i] );
	}

    intermediate_A.resize(n_outports_A);
    output_msgs_A.resize(n_outports_A);
    for( uint i = 0; i < n_outports_A; i++ ) {
        intermediate_A[i].resize( outport_dimensions_A[i] );
        output_msgs_A[i].values.resize( outport_dimensions_A[i] );
    }
    
    log(Info) << "EtherCATwrite (" << PARTNAME << ")::AddAnalogOuts: Added AnalogIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    
	return;
}

void EtherCATwrite::AddDigitalOuts(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;
	
	log(Info) << "EtherCATwrite (" << PARTNAME << ")::ADDING: [ " << INPORT_DIMENSIONS.size() << " , " << OUTPORT_DIMENSIONS.size() << " , " << FROM_WHICH_INPORT.size() << " , " << FROM_WHICH_ENTRY.size() << " ]!" << endlog();

	//! Check configuration	
	// Check if the DigitalIns is already added for this bodypart
	for(uint l = 0; l < added_bodyparts_D.size(); l++) {
		if (PARTNAME == added_bodyparts_D[l]) {
			log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. The number of entries in from_which_inport_D and from_which_entry_D should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_D+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_D!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1-n_inports_D] || FROM_WHICH_ENTRY[j] < 0 ) {
            log(Error) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Could not add DigitalOut. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
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
    input_D.resize(n_inports_D);
    for( uint i = 0; i < n_inports_D; i++ ) {
        input_D[i].resize( inport_dimensions_D[i] );
	}

    output_msgs_D.resize(n_outports_D);
    for( uint i = 0; i < n_outports_D; i++ ) {
        intermediate_D[i].resize(outport_dimensions_D[i]);
        output_msgs_D[i].values.resize(outport_dimensions_D[i]);
    }
    
    log(Info) << "EtherCATwrite (" << PARTNAME << ")::AddDigitalOuts: Succesfully added DigitalIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();

	return;
}

//! Functions to edit inputs
// Analog
void EtherCATwrite::AddAddition_A(int ID, doubles VALUES)
{
	// Check
	if( ID <= 0 || ID > n_outports_A) {
		log(Error) << "EtherCATwrite::AddAddition_A: Could not add addition. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_A << "!" << endlog();
		return;
	}
	if( VALUES.size() != output_msgs_A[ID-1].values.size() ) {
		log(Error) << "EtherCATwrite::AddAddition_A: Could not add addition. Invalid size of VALUES. Should have be of size :" << output_msgs_A[ID-1].values.size() << "!" << endlog();
		return;
	}
	if( multiply_status_A[ID-1] ) {
		log(Error) << "EtherCATwrite::AddAddition_A: Could not add addition. For this output a multiplier is already there" << endlog();
		log(Error) << "If you want to do both a multiply and an addition, then do the addition first and then the mulitiply" << endlog();
		return;
	}
	
	// Resize
	addition_values_A[ID-1].resize(output_msgs_A[ID-1].values.size());
	
	// Save ramp properties
	for( uint i = 0; i < output_msgs_A[ID-1].values.size(); i++ ) {
		addition_values_A[ID-1][i] = VALUES[i];
	}
	
	if( addition_status_A[ID-1] ) {
		log(Warning) << "EtherCATwrite::AddAddition_A: Overwritten existing addition!" << endlog();
	} else {
		log(Warning) << "EtherCATwrite::AddAddition_A: Succesfully added addition!" << endlog();
	}
	
	// Set status
	addition_status_A[ID-1] = true;
}

void EtherCATwrite::AddMultiply_A(int ID, doubles FACTOR)
{
	// Check
	if( ID <= 0 || ID > n_outports_A) {
		log(Error) << "EtherCATwrite::AddMultiply_A: Could not add multiplier. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_A << "!" << endlog();
		return;
	}
	if( FACTOR.size() != output_msgs_A[ID-1].values.size() ) {
		log(Error) << "EtherCATwrite::AddMultiply_A: Could not add multiplier. Invalid size of FACTOR. Should have be of size :" << output_msgs_A[ID-1].values.size() << "!" << endlog();
		return;
	}
	
	// Resize
	multiply_factor_A[ID-1].resize(output_msgs_A[ID-1].values.size());
	
	// Save ramp properties
	for( uint i = 0; i < output_msgs_A[ID-1].values.size(); i++ ) {
		multiply_factor_A[ID-1][i] = FACTOR[i];
	}
	
	if( addition_status_A[ID-1] ) {
		log(Warning) << "EtherCATwrite::AddMultiply_A: Overwritten existing multiplier!" << endlog();
	} else {
		log(Warning) << "EtherCATwrite::AddMultiply_A: Succesfully added multiplier!" << endlog();
	}
	
	// Set status
	multiply_status_A[ID-1] = true;
	
}

void EtherCATwrite::AddMatrixTransform_A(int ID, double INPUTSIZE, double OUTPUTSIZE)
{
	// Check
	if( matrixtransform_status_A[ID-1] ) {
		log(Warning) << "EtherCATwrite::AddMatrixTransform_A: Could not add Matrix Transform, since this already excists for this bodypart. Overwriting is not supported at the moment" << endlog();
		return;
	}
	if( ID <= 0 || ID > n_outports_A) {
		log(Error) << "EtherCATwrite::AddMatrixTransform_A: Could not add matrix transform. Invalid ID: " << ID << ".  1 <= ID <= " << n_outports_A << "!" << endlog();
		return;
	}
	if( INPUTSIZE != inport_dimensions_A[ID-1]) { 
		if( INPUTSIZE < inport_dimensions_A[ID-1]) { 	// smaller matrix then nr of inputs is allowed, not recommended
			log(Warning) << "EtherCATwrite::AddMatrixTransform_A: INPUTSIZE: " << INPUTSIZE << " is smaller than the size of the inport inport_dimensions_A[ID-1]:" << inport_dimensions_A[ID-1] << "!" << endlog();
		} else { 										// larger matrix then nr of inputs is not allowed
			log(Error) << "EtherCATwrite::AddMatrixTransform_A: Could not add matrix transform. INPUTSIZE: " << INPUTSIZE << " is larger than the size of the inport inport_dimensions_A[ID-1]:" << inport_dimensions_A[ID-1] << "!" << endlog();
			return;
		}
	}
	if( OUTPUTSIZE != outport_dimensions_A[ID-1]) {
		if( OUTPUTSIZE < outport_dimensions_A[ID-1]) { 	// smaller matrix then nr of inputs is allowed, not recommended
			log(Warning) << "EtherCATwrite::AddMatrixTransform_A: INPUTSIZE: " << OUTPUTSIZE << " is smaller than the size of the outport outport_dimensions_A[ID-1]:" << outport_dimensions_A[ID-1] << "!" << endlog();
		} else { 										// larger matrix then nr of inputs is not allowed
			log(Error) << "EtherCATwrite::AddMatrixTransform_A: Could not add matrix transform. INPUTSIZE: " << OUTPUTSIZE << " is larger than the size of the outport outport_dimensions_A[ID-1]:" << inport_dimensions_A[ID-1] << "!" << endlog();
			return;
		}
	}
	
	// Update outport_dimensions_E
	outport_dimensions_A[ID-1] = OUTPUTSIZE;
		
	// Add property to store Matrix
	matrixtransform_A[ID-1].resize(outport_dimensions_A[ID-1]);
	for ( uint i = 0; i < outport_dimensions_A[ID-1]; i++ ) {
		
		matrixtransform_A[ID-1][i].resize(outport_dimensions_A[ID-1]); 
		string name = added_bodyparts_A[ID-1]+"_matrixtransform"+to_string(i+1);
		addProperty( name, matrixtransform_A[ID-1][i]);
		
		// Set matrix default to Identity matrix
		for ( uint l = 0; l < matrixtransform_A[ID-1][i].size(); l++ ) { 
			matrixtransform_A[ID-1][i][l] = 0.0;
		}
		matrixtransform_A[ID-1][i][i] = 1.0;
	}
	
	if( matrixtransform_status_A[ID-1] ) {
		log(Warning) << "EtherCATwrite::AddMatrixTransform_A: Overwritten existing matrix transform." << endlog();
	} else {
		log(Warning) << "EtherCATwrite::AddMatrixTransform_A: Added a matrix transform." << endlog();
	}
	
	// Set status
	matrixtransform_status_A[ID-1] = true;
	
}

void EtherCATwrite::CheckAllConnections()
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
				log(Error) << "EtherCATwrite: Analog inport " << inports_A[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_A; i++) {
			if ( !outports_A[i].connected() ) {
				log(Info) << "EtherCATwrite: Analog outport " << outports_A[i].getName() << " is not connected." << endlog();
			}
		}

		// DigitalIns
		for(uint i = 0; i < n_inports_D; i++) {
			if ( !inports_D[i].connected() ) {
				log(Error) << "EtherCATwrite: Digital inport " << inports_D[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_D; i++) {
			if ( !outports_D[i].connected() ) {
				log(Info) << "EtherCATwrite: Digital outport " << outports_D[i].getName() << " is not connected." << endlog();
			}
		}
	}
	
}

void EtherCATwrite::Calculate_A()
{	
	// Addition
    for( uint i = 0; i < n_outports_A; i++ ) {
		if (addition_status_A[i]) {
			for( uint k = 0; k < output_msgs_A[i].values.size(); ++k) {
				intermediate_A[i][k] += addition_values_A[i][k];
			}
		}
    }
	
	// Multiplier
    for( uint i = 0; i < n_outports_A; i++ ) {
		if (multiply_status_A[i]) {
			for( uint k = 0; k < output_msgs_A[i].values.size(); ++k) {
				intermediate_A[i][k] = intermediate_A[i][k]*multiply_factor_A[i][k];
			}
		}
    }
    
	// Matrix Transform	
    for( uint i = 0; i < n_outports_A; i++ ) {
		if (matrixtransform_status_A[i]) {
						
			// Store output of MapInputs2Outputs operation into a temporary input vector
			doubles input_MT_A = intermediate_A[i];
			
			// Resize output vectors in case of a non square Matrix transformation
			output_msgs_A[i].values.resize(outport_dimensions_A[i]);
						
			for ( uint k = 0; k < output_msgs_A[i].values.size(); k++ ) {
				intermediate_A[i][k] = 0.0;
				output_msgs_A[i].values[k] = 0.0;
				
				for ( uint l = 0; l < input_MT_A.size(); l++ ) {
					output_msgs_A[i].values[k] += matrixtransform_A[i][k][l] * input_MT_A[l];
					//log(Warning) << "EtherCATwrite: -> [" << output_msgs_A[i].values[k] << " = " << matrixtransform_A[i][k][l] << "*" << input_MT_A[l] << "]" << endlog();
				}
			}
		}
	}
		
	// Convert to msg (only for outputs that do not have a matrix transform operation)
	for( uint i = 0; i < n_outports_A; i++ ) {								
		if (!matrixtransform_status_A[i]) {
			for ( uint k = 0; k < output_msgs_A[i].values.size(); k++ ) {
				output_msgs_A[i].values[k] = intermediate_A[i][k]; 
			}
		}
	}
	
}

void EtherCATwrite::Calculate_D()
{
	// Convert to msg
	for( uint i = 0; i < n_outports_D; i++ ) {								
		for ( uint k = 0; k < output_msgs_D[i].values.size(); k++ ) {
			output_msgs_D[i].values[k] = intermediate_D[i][k]; 
		}
	}
	
}

void EtherCATwrite::ReadInputs()
{
    for( uint i = 0; i < n_inports_A; i++ ) {
		inports_A[i].read(input_A[i]);
    }
    for( uint i = 0; i < n_inports_D; i++ ) {
		inports_D[i].read(input_D[i]);
    }
}

void EtherCATwrite::WriteOutputs()
{	
    for( uint i = 0; i < n_outports_A; i++ ) {
		outports_A[i].write(output_msgs_A[i]);
    }
    for( uint i = 0; i < n_outports_D; i++ ) {
		outports_D[i].write(output_msgs_D[i]);
    }
}

void EtherCATwrite::MapInputs2Outputs()
{
	// Do mapping of input entries on into output
    uint j = 0;
    for( uint i = 0; i < n_outports_A; i++ ) {
        for( uint k = 0; k < outport_dimensions_A[i]; ++k) {
			if (from_which_inport_A[j] == 0 || from_which_entry_A[j] ==0) {
				intermediate_A[i][k] = 0.0;
			} else {			
				intermediate_A[i][k] = input_A[ from_which_inport_A[j]-1 ][ from_which_entry_A[j]-1 ];
			}
			
			j++;
        }
    }

	// Do mapping of input entries on into output
    j = 0;
    for( uint i = 0; i < n_outports_D; i++ ) {
        for( uint k = 0; k < outport_dimensions_D[i]; ++k) {
			if (from_which_inport_A[j] == 0 || from_which_entry_A[j] ==0) {
				intermediate_D[i][k] = 0;
			} else {			
				intermediate_D[i][k] = input_D[ from_which_inport_D[j]-1 ][ from_which_entry_D[j]-1 ];
			}
			
			j++;            
        }
    }
}

ORO_CREATE_COMPONENT(ETHERCATWRITE::EtherCATwrite)
