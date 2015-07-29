#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ADIns.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

ADIns::ADIns(const string& name) : TaskContext(name, PreOperational)
{
	//! Operations
    addOperation("AddAnalogIns", &ADIns::AddAnalogIns, this, OwnThread)
            .doc("Add one or more analog ins")
            .arg("INPORT_DIMENSIONS","Array containing for each inport an entry with value the size of that input port")
            .arg("OUTPORT_DIMENSIONS","Array containing for each outport an entry with value the size of that output port")
            .arg("FROM_WHICH_INPORT","Array specifying where the input from the inports should go - first specify from which inport")
            .arg("FROM_WHICH_ENTRY","Array specifying where the input from the inports should go - second specify which entry");
}
ADIns::~ADIns(){}

bool ADIns::configureHook()
{
	//! init
	
	// Global
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	goodToGO = false;
	
	// Analog
	n_inport_entries_A = 0;
	n_outport_entries_A = 0;
	n_inports_A = 0;
	n_outports_A = 0;
	
	// Digital
	n_inports_D = 0;
	n_outports_D = 0;

}

bool ADIns::startHook(){}

void ADIns::updateHook()
{
	//! 8 seconds after boot, Check all connections
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 4.0)) {
		goodToGO = true;

		// AnalogIns
		for(uint i = 0; i < n_inports_A; ++i) {
			if ( !inports_A[i].connected() ) {
				log(Error) << "ADIns: Analog inport " << inports_A[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_A; ++i) {
			if ( !outports_A[i].connected() ) {
				log(Error) << "ADIns: Analog outport " << outports_A[i].getName() << " is not connected." << endlog();
			}
		}
		
		// DigitalIns
		for(uint i = 0; i < n_inports_D; ++i) {
			if ( !inports_D[i].connected() ) {
				log(Warning) << "ADIns: Digital inport " << inports_D[i].getName() << " is not connected." << endlog();
			}
		}
		for(uint i = 0; i < n_outports_D; ++i) {
			if ( !outports_D[i].connected() ) {
				log(Warning) << "ADIns: Digital outport " << outports_D[i].getName() << " is not connected." << endlog();
			}
		}
	}

	//! AnalogIns	
    // Initialize in- and outport messages
	std::vector< soem_beckhoff_drivers::AnalogMsg > inputdata_msgs;
    std::vector< doubles > outputdata;
    
    // Resizing in- and outport messages
    inputdata_msgs.resize(n_inports_A);	
    for( uint i = 0; i < n_inports_A; ++i ) {
        inputdata_msgs[i].values.resize( inport_dimensions_A[i] );
	}
    outputdata.resize(n_outports_A);
    for( uint i = 0; i < n_outports_A; i++ ) {
        outputdata[i].resize( outport_dimensions_A[i] );
    }
    
    // Read input ports
    for( uint i = 0; i < n_inports_A; ++i ) {
		inports_A[i].read(inputdata_msgs[i]);
    }

	// Do mapping of input entries on into outputdata and write to port
    uint k = 0;
    for( uint i = 0; i < n_outports_A; ++i )
    {
        for( uint j = 0; j < outport_dimensions_A[i]; ++j)
        {
            outputdata[i][j] = inputdata_msgs[ from_which_inport_A[k]-1 ].values[ from_which_entry_A[k]-1 ];
            ++k;
        }
        outports_A[i].write(outputdata[i]);
    }
}

void ADIns::AddAnalogIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY)
{
	// Init 
	uint N_INPORTS = INPORT_DIMENSIONS.size();
	uint N_OUTPORTS = OUTPORT_DIMENSIONS.size();
	uint N_INPORT_ENTRIES = 0;
	uint N_OUTPORT_ENTRIES = 0;

	//! Check configuration	
	// Check for invalid number of ports
    if(N_INPORTS < 1 || N_INPORTS > MAX_PORTS) {
        log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. Invalid number of inports: " << N_INPORTS << "!" << endlog();
        return;
    }
    if(N_OUTPORTS < 1 || N_OUTPORTS > MAX_PORTS) {
        log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. Invalid number of outports: " << N_OUTPORTS << "!" << endlog();
        return;
    }

	// Count the number of entries respectively for N_INPORT_ENTRIES and N_OUTPORT_ENTRIES
    for(uint i = 0; i < N_INPORTS; ++i) {
        if(INPORT_DIMENSIONS[i] < 1) {
            log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. Inport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_INPORT_ENTRIES += INPORT_DIMENSIONS[i];
    }
    for(uint i = 0; i < N_OUTPORTS; ++i) {
        if(OUTPORT_DIMENSIONS[i] < 1) {
            log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. Outport_dimensions cannot contain value smaller than one!" << endlog();
            return;
        }
        N_OUTPORT_ENTRIES += OUTPORT_DIMENSIONS[i];
    }

	// Check if the total number of entries matches the size of FROM_WHICH_INPORT and FROM_WHICH_ENTRY
    if ((FROM_WHICH_INPORT.size() != N_OUTPORT_ENTRIES) || (FROM_WHICH_ENTRY.size() != N_OUTPORT_ENTRIES)) {
        log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. The number of entries in from_which_inport_A and from_which_entry_A should equal the total number of output values." << endlog();
        return;
    }

	// Check validity of each entry in the FROM_WHICH_INPORT and FROM_WHICH_ENTRY 
    for(uint i = 0; i < N_OUTPORT_ENTRIES; ++i) {
        if( FROM_WHICH_INPORT[i] > (n_inports_A+N_INPORTS) || FROM_WHICH_INPORT[i] <= 0 ) {
            log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. From_which_inport array contains port no. " << FROM_WHICH_INPORT[i] << " which does not exist according to inport_dimensions_A!" << endlog();
            return;
        }
        else if ( FROM_WHICH_ENTRY[i] > INPORT_DIMENSIONS[ FROM_WHICH_INPORT[i]-1-n_inports_A] || FROM_WHICH_ENTRY[i] <= 0 ) {
            log(Error) << "ADIns::AddAnalogIns: Could not add AnalogIn. From_which_entry array contains entry no. " << FROM_WHICH_ENTRY[i] << " which does not exist for inport no. " << FROM_WHICH_INPORT[i] << "!" << endlog();
            return;
        }
    }
    
    //! Now that all inputs have been properly examined. The in- and outports can be created
    for( uint i = n_inports_A; i < (n_inports_A+N_INPORTS); i++ ) {
		addEventPort( "Ain"+to_string(i+1), inports_A[i] );
	}
    for( uint i = n_outports_A; i < (n_outports_A+N_OUTPORTS); i++ ) {
        addPort( "Aout"+to_string(i+1), outports_A[i] );
    }
    
    //! And the temperary properties can be added to the global properties
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
	for( uint i = 0; i < N_OUTPORT_ENTRIES; i++ ) {
		from_which_inport_A.push_back((int) FROM_WHICH_INPORT[i]);
	}
	for( uint i = 0; i < N_OUTPORT_ENTRIES; i++ ) {
		from_which_entry_A.push_back((int) FROM_WHICH_ENTRY[i]);
	}
    
    log(Warning) << "ADIns::AddAnalogIns: Succesfully added AnalogIns with " << N_INPORTS << " inports and " << N_OUTPORTS << " outports!" << endlog();
    log(Warning) << "ADIns::AddAnalogIns: Now we have a total of " << n_inports_A << " inports and " << n_outports_A << " outports!" << endlog();
    log(Warning) << "ADIns::AddAnalogIns: With a total of " << n_inport_entries_A << " input entries and " << n_outport_entries_A << " output entries!" << endlog();
    
	return;
}

void ADIns::AddDigitalIns(int N_DIGITAL_INS, doubles FLIP_OUT)
{
	//! Checking of input properties	
	// Check if the number of to be added digital ins lies between one and MAX_PORTS
    if(N_DIGITAL_INS < 1 || N_DIGITAL_INS > MAX_PORTS) {
        log(Error) << "ADIns::AddDigitalIns: Could not add AnalogIn. Invalid size of inport: " << N_DIGITAL_INS << "!" << endlog();
        return;
    }
    
    // Add number of in and outputs respectively to N_INPORTS and N_OUTPORTS
    if(FLIP_OUT.size() != N_DIGITAL_INS) {
        log(Error) << "ADIns::AddDigitalIns: Could not add AnalogIn. Invalid size of FLIP_OUT: " << FLIP_OUT.size() << ". Should match N_DIGITAL_INS:" << N_DIGITAL_INS << "!" << endlog();
        return;
    }   
    
	//! Add DigitalIn
	// Add ports
	addPort( "Din"+to_string(n_inports_D+1), inports_D[n_inports_D] );
	for ( uint i = (n_outports_D); i < (n_outports_D+N_DIGITAL_INS); i++ ) {
		addPort("Dout"+to_string(i+1), outports_D[i]);
	}
	
	// Update flip
	flip_D.resize(n_outports_D);
	for ( uint i = (n_outports_D); i < (n_outports_D+N_DIGITAL_INS); i++ ) {
		flip_D[n_outports_D+i] = FLIP_OUT[i];
	}
	
	// Update the global properties
	n_inports_D++;
	n_outports_D += N_DIGITAL_INS;

	log(Warning) << "ADIns::AddDigitalIns: Succesfully added DigitalIns with one inport of size " << N_DIGITAL_INS << "!" << endlog();	
	return;
}

ORO_CREATE_COMPONENT(SOEM::ADIns)
