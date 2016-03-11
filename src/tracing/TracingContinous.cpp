/** TracingContinous.cpp
 *
 * @class TracingContinous
 *
 * \author Max Baeten
 * \date Feb, 2016
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <fstream>

#include "TracingContinous.hpp"

using namespace std;
using namespace RTT;
using namespace Signal;

TracingContinous::TracingContinous(const string& name) : 
											TaskContext(name, PreOperational),
											filename("data.txt"),
											buffersize(16384),
											Ts(0.001)
{
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
	addProperty( "sendErrorLog_delay", sendErrorLog_delay ).doc("After an error is detected, a delay is used before sending the error log");

	addOperation("AddBodypart", &TracingContinous::AddBodypart, this, OwnThread)
		.doc("Add bodypart to trace")
		.arg("PARTNAME","Name of the bodypart")
		.arg("BPID","ID number of the bodypart")
		.arg("NRPORTS","Nr Ports")
		.arg("NRJOINTS","Number of joints")
		.arg("PORTNAMES","To label the ports that are traced");
}

TracingContinous::~TracingContinous(){}

bool TracingContinous::configureHook()
{
	// Init
	n_totalports = 0;
	
	for ( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		buffer_status[l] = false;
		buffer_nrports[l] = 0;
		buffer_nrjoints[l] = 0;
	}
	
	return true;
}

bool TracingContinous::startHook()
{
	n_cyclicbuffer = 0;
	error = false;
	buffer_full = false;
	error_bpid = 0;
	sendErrorLog_delaycntr =0;
	
	return true;
}

void TracingContinous::updateHook()
{
	// First updatehook is useless
	if ( !this->isRunning() ) {    // if(counter == -1){counter = 0;return;}
		return;
	}
	
	// Check for errors sendErrorLog_delay
	if (!error) {
		for ( uint i = 0; i < MAX_BODYPARTS; i++ ) {
			errorInports[i].read(error);
			if (error == true ) {
				error_bpid = i+1;
				log(Warning) << "TracingContinous: Detected the Error!" << endlog();
			}
		}
	} else {
		sendErrorLog_delaycntr++;
		if (sendErrorLog_delaycntr>sendErrorLog_delay) {
			stopHook(error_bpid,n_cyclicbuffer);
		}
	}
	
	// Storing data in cyclic buffer
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		if (buffer_status[l] == true) {
			for( uint i = 0; i < buffer_nrports[l]; i++ ) {
				if ( dataInports[l][i].read( input[l][i] ) == NewData ) {
					for ( uint k = 0; k < buffer_nrjoints[l]; k++ ) {
						// to do fix this segfault
						buffer[l][i][k][n_cyclicbuffer] = input[l][i][k];
					}
				} else { // no new data then fill with -1
					for ( uint k = 0; k < buffer_nrjoints[l]; k++ ) {
						buffer[l][i][k][n_cyclicbuffer] = -1.0;
					}
				}
			}
		}
	}
	
	// update cyclic buffer
	n_cyclicbuffer++;
	if (n_cyclicbuffer==buffersize) {
		buffer_full = true;
		n_cyclicbuffer = 0;
	}
}

void TracingContinous::AddBodypart(string PARTNAME, uint BPID, uint NRPORTS, uint NRJOINTS, strings PORTNAMES)
{
	// Check BPID, NRPORTS, NRJOINTS
	if( BPID <= 0 || BPID > MAX_BODYPARTS) {
		log(Error) << "TracingContinous::AddBodypart(" << PARTNAME << "): Could not add bodypart. Invalid BPID: " << BPID << "! Should be between 0 and " << MAX_BODYPARTS << "!" << endlog();
		return;
	}
	if( NRPORTS <= 0 || NRPORTS > MAX_PORTS) {
		log(Error) << "TracingContinous::AddBodypart(" << PARTNAME << "): Could not add bodypart. Invalid NRPORTS: " << NRPORTS << "! Should be between 0 and " << MAX_PORTS << "!" << endlog();
		return;
	}
	if( NRJOINTS <= 0 || NRJOINTS > MAX_JOINTS) {
		log(Error) << "TracingContinous::AddBodypart(" << PARTNAME << "): Could not add bodypart. Invalid NRJOINTS: " << NRJOINTS << "! Should be between 0 and " << MAX_JOINTS << "!" << endlog();
		return;
	}
	if( PORTNAMES.size() != NRPORTS ) {
		log(Error) << "TracingContinous::AddBodypart(" << PARTNAME << "): Could not add bodypart. Invalid size of PORTNAMES: " << PORTNAMES.size() << "! Should be equal to NRPORTS: " << NRPORTS << "!" << endlog();
		return;
	}
	
	// Init
	buffer_status[BPID-1] = true;
	buffer_nrports[BPID-1] = NRPORTS;
	buffer_nrjoints[BPID-1] = NRJOINTS;
	n_totalports += NRPORTS;
	n_totaltraces += NRPORTS*NRJOINTS;
	
	// Add ports and append buffer_name for every joint of every port
	addPort(PARTNAME+"_error", errorInports[BPID-1]);
	for ( uint i = 0; i < NRPORTS; i++ ) { 
		string portname = PARTNAME+"_"+PORTNAMES[i]; 
		addPort(portname, dataInports[BPID-1][i]);
		
		for ( uint k = 0; k < NRJOINTS; k++ ) {
			buffer_names.push_back(portname+to_string(k));
		}
	}
	
	// Create data structures 
	buffer[BPID-1].resize(NRPORTS);
	input[BPID-1].resize(NRPORTS);
	for ( uint i = 0; i < NRPORTS; i++ ) { 
		buffer[BPID-1][i].resize(NRJOINTS);
		input[BPID-1][i].resize(NRJOINTS);
		for ( uint k = 0; k < NRPORTS; k++ ) {
			buffer[BPID-1][i][k].resize(buffersize);
		}
	}
	
	log(Warning) << "TracingContinous: AddedBodypart: " << PARTNAME <<"!" << endlog();
}

void TracingContinous::stopHook(int BPID, uint N_CYCLICBUFFER)
{
	if (!buffer_full) {
		log(Warning) << "TracingContinous: Detected error for BPID: " << BPID <<", but buffer was not yet full so auto log is skipped!" << endlog();
	} else {
		sendErrorLog(BPID,N_CYCLICBUFFER);
	}
	
	startHook();
	return;
}

void TracingContinous::sendErrorLog(int BPID, uint N_CYCLICBUFFER)
{
	// Construct log file
	FILE * pFile;
	pFile = fopen (filename.c_str(),"w");
	
	// First add all the portnames on top of the file
	fprintf(pFile, "Time    \t");
	for ( uint m = 0; m < n_totaltraces; m++ ) {
		fprintf(pFile, "%s    \t", buffer_names[m].c_str());
	}
	fprintf(pFile, "\n");
	
	// Start at N_CYCLICBUFFER+1 and loop to end of buffer, at end start at 0 and continue to N_CYCLICBUFFER 
	for ( uint n = N_CYCLICBUFFER+1; n==N_CYCLICBUFFER; n++ ) {
		for ( uint i = 0; i < buffer_nrports[BPID-1]; i++ ) {
			for ( uint k = 0; k < buffer_nrjoints[BPID-1]; k++ ) {
				fprintf(pFile, "%f    \t", buffer[BPID-1][i][k][n] );
			}
		}
		// At the end of the buffer go back to n=0
		if (n==buffersize) {
			n=0;
		}
	}
	
	log(Warning) << "TracingContinous: Trace written!" << endlog();
	fclose(pFile);
}

ORO_CREATE_COMPONENT(Signal::TracingContinous)
