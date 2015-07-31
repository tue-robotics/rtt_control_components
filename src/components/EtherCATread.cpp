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

		// AnalogIns
		for(uint i = 0; i < n_inports_A; i++) {
			if ( !inports_A[i].connected() ) {
				log(Error) << "EtherCATread: Analog inport " << inports_A[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_A; i++) {
			if ( !outports_A[i].connected() ) {
				log(Info) << "EtherCATread: Analog outport " << outports_A[i].getName() << " is not connected." << endlog();
			}
		}
		
		// DigitalIns
		for(uint i = 0; i < n_inports_D; i++) {
			if ( !inports_D[i].connected() ) {
				log(Error) << "EtherCATread: Digital inport " << inports_D[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_D; i++) {
			if ( !outports_D[i].connected() ) {
				log(Info) << "EtherCATread: Digital outport " << outports_D[i].getName() << " is not connected." << endlog();
			}
		}
		
		// EncoderIns
		for(uint i = 0; i < n_inports_E; i++) {
			if ( !inports_E[i].connected() ) {
				log(Error) << "EtherCATread: Encoder inport " << inports_E[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_E; i++) {
			if ( !outports_E[i].connected() ) {
				log(Info) << "EtherCATread: Encoder outport " << outports_E[i].getName() << " is not connected." << endlog();
			}
		}
	}

	//! AnalogIns
    // Read input ports
    for( uint i = 0; i < n_inports_A; i++ ) {
		inports_A[i].read(input_msgs_A[i]);
    }

	// Do mapping of input entries on into output and write to port
    uint j = 0;
    for( uint i = 0; i < n_outports_A; i++ ) {
        for( uint k = 0; k < outport_dimensions_A[i]; ++k) {
            output_A[i][k] = input_msgs_A[ from_which_inport_A[j]-1 ].values[ from_which_entry_A[j]-1 ];
            ++j;
        }
        outports_A[i].write(output_A[i]);
    }
    
    //! DigitalIns
    // Read input ports
    for( uint i = 0; i < n_inports_D; i++ ) {
		inports_D[i].read(input_msgs_D[i]);
    }

	// Do mapping of input entries on into output and write to port
    j = 0;
    for( uint i = 0; i < n_outports_D; i++ ) {
        for( uint k = 0; k < outport_dimensions_D[i]; ++k) {
            output_D[i][k] = input_msgs_D[ from_which_inport_D[j]-1 ].values[ from_which_entry_D[j]-1 ];
            ++j;
        }
        outports_D[i].write(output_D[i]);
    }
    
    //! EncoderIns
    // Read input ports
    for( uint i = 0; i < n_inports_E; i++ ) {
		inports_E[i].read(input_msgs_E[i]);
    }

	// Do mapping of input entries on into output and write to port
    j = 0;
    for( uint i = 0; i < n_outports_E; i++ ) {
        for( uint k = 0; k < outport_dimensions_E[i]; ++k) {
            output_E[i][k] = input_msgs_E[ from_which_inport_E[j]-1 ].value;
            ++j;
        }
        outports_E[i].write(output_E[i]);
    }
	
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
    for(uint j = 0; j < N_OUTPORT_ENTRIES; ++j) {
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
    for( uint i = 0; i < N_INPORTS; i++ ) {
		addEventPort( PARTNAME+"_Ain"+to_string(i+1), inports_A[i] );
	}
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Aout"+to_string(i+1), outports_A[i] );
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
    for( uint i = 0; i < N_INPORTS; i++ ) {
		addEventPort( PARTNAME+"_Din"+to_string(i+1), inports_A[i] );
	}
    for( uint i = 0; i < N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Dout"+to_string(i+1), outports_A[i] );
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
			log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add AnalogIn. There is already an AnalogIn for bodypart " << PARTNAME << "!" << endlog();
			return;
		}
	}
    
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; i++) {
        if(INPORT_DIMENSIONS[i] != 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. All inport_dimensions should be be 1!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; i++) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. The number of entries in from_which_inport_E and from_which_entry_E should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint j = 0; j < N_OUTPORT_ENTRIES; j++) {
        if( FROM_WHICH_INPORT[j] > (n_inports_E+N_INPORTS) || FROM_WHICH_INPORT[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. From_which_inport array contains port no. " << FROM_WHICH_INPORT[j] << " which does not exist according to inport_dimensions_E!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[j] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[j]-1] || FROM_WHICH_ENTRY[j] <= 0 ) {
            log(Error) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Could not add EncoderIn. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[j] << " which does not exist for inport no. " << FROM_WHICH_INPORT[j] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
    for( uint i = n_inport_entries_E; i < n_inport_entries_E+N_INPORTS; i++ ) {
		addEventPort( PARTNAME+"_Ein"+to_string(i+1-n_inport_entries_E), inports_E[i] );
	}
    for( uint i = n_outport_entries_E; i < n_outport_entries_E+N_OUTPORTS; i++ ) {
        addPort( PARTNAME+"_Eout"+to_string(i+1-n_inport_entries_E), outports_E[i] );
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
    // input message size for an encoder can only be one so no need to resize here
    output_E.resize(n_outports_E);
    for( uint i = 0; i < n_outports_E; i++ ) {
        output_E[i].resize( outport_dimensions_E[i] );
    }
    
    log(Warning) << "EtherCATread (" << PARTNAME << ")::AddEncoderIns: Succesfully added EncoderIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    
	return;
}

ORO_CREATE_COMPONENT(ETHERCATREAD::EtherCATread)
