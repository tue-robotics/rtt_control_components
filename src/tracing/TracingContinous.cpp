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
#include <stdlib.h>

#include "TracingContinous.hpp"

using namespace std;
using namespace RTT;
using namespace Signal;

TracingContinous::TracingContinous(const string& name) : 
											TaskContext(name, PreOperational),
											loglocation("/home/amigo/ros/data/private/hardware_tracing/autolog"),
											buffersize(16384),
											Ts(0.001)
{
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "loglocation", loglocation ).doc("Name of the location of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
	addProperty( "sendErrorLog_delay", sendErrorLog_delay ).doc("After an error is detected, a delay is used before sending the error log");

	addOperation("AddBodypart", &TracingContinous::AddBodypart, this, OwnThread)
		.doc("Add bodypart to trace")
		.arg("PARTNAME","Name of the bodypart")
		.arg("BPID","ID number of the bodypart")
		.arg("NRPORTS","Nr Ports")
		.arg("NRJOINTS","Number of joints")
		.arg("PORTNAMES","To label the ports that are traced");
	addOperation("sendLog", &TracingContinous::sendLog, this, OwnThread)
		.doc("Add bodypart to trace")
		.arg("BPID","BodypartID of which the log file should be saved");
}

TracingContinous::~TracingContinous(){}

bool TracingContinous::configureHook()
{
	// Init
	n_totalports = 0;
	n_totaltraces = 0;
	processingerror = false;
	
	for ( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		buffer_status[l] = false;
		buffer_nrports[l] = 0;
		buffer_nrjoints[l] = 0;
		errors[l] = false;
	}
	
	return true;
}

bool TracingContinous::startHook()
{
	n_cyclicbuffer = 0;
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
	if (!processingerror) {
		for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
			if (buffer_status[l] == true) {
				bool error;
				errorInports[l].read(error);
				if (error == true ) {
					processingerror = true;
					errors[l] = true;
					error_bpid = l+1;
				}
			}
		}
	} else {
		sendErrorLog_delaycntr++;
		if (sendErrorLog_delaycntr > sendErrorLog_delay) {
			processingerror = false;
			stopHook(error_bpid,n_cyclicbuffer);
		}
	}
	
	// Storing data in cyclic buffer
	for( uint l = 0; l < MAX_BODYPARTS; l++ ) {
		if (buffer_status[l] == true) {
			for( uint i = 0; i < buffer_nrports[l]; i++ ) {
				if ( dataInports[l][i].read( input[l][i] ) == NewData ) {
					for ( uint k = 0; k < buffer_nrjoints[l]; k++ ) {
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
			buffer_names[BPID-1].push_back(portname+to_string(k));
		}
	}
	
	// Create data structures 
	buffer[BPID-1].resize(NRPORTS);
	input[BPID-1].resize(NRPORTS);
	for ( uint i = 0; i < NRPORTS; i++ ) { 
		buffer[BPID-1][i].resize(NRJOINTS);
		input[BPID-1][i].resize(NRJOINTS);
		for ( uint k = 0; k < NRJOINTS; k++ ) {
			buffer[BPID-1][i][k].resize(buffersize);
		}
	}
	
	log(Warning) << "TracingContinous: n ports -> 		buffer[BPID-1].size(): " << buffer[BPID-1].size() <<"!" << endlog();
	log(Warning) << "TracingContinous: n_joints ->  	buffer[BPID-1][0].size(): " << buffer[BPID-1][0].size() <<"!" << endlog();
	log(Warning) << "TracingContinous: n datapoints -> 	buffer[BPID-1][0][0].size(): " << buffer[BPID-1][0][0].size() <<"!" << endlog();
}

void TracingContinous::stopHook(int BPID, uint N_CYCLICBUFFER)
{
	log(Warning) << "TracingContinous: stopHook for BPID: " << BPID << ",  and N_CYCLICBUFFER: " << N_CYCLICBUFFER << " !" << endlog();	
	
	// Construct log file
	time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	string filename = loglocation+"/autolog"+buf+".dat";
	FILE* pFile;	
	pFile = fopen (filename.c_str(),"w");
	
	// First add all the portnames on top of the file
	fprintf(pFile, "Time  \t");
	for ( uint m = 0; m < buffer_names[BPID-1].size(); m++ ) {
		fprintf(pFile, "|%s    \t", buffer_names[BPID-1][m].c_str());
	}
	fprintf(pFile, "\n");
	
	uint n = N_CYCLICBUFFER+1;
	// Start at N_CYCLICBUFFER+1 and loop to end of buffer, at end start at 0 and continue to N_CYCLICBUFFER 
	for ( uint nn = 0; nn<buffersize; nn++ ) {
		
		fprintf(pFile, "%If    \t", (nn*Ts));
		// Write line of data
		for ( uint i = 0; i < buffer_nrports[BPID-1]; i++ ) {
			for ( uint k = 0; k < buffer_nrjoints[BPID-1]; k++ ) {
				fprintf(pFile, "%If    \t", buffer[BPID-1][i][k][n] );
			}
		}
		fprintf(pFile, "\n");
		
		// Increas n and at the end of the buffer go back to n=0
		n++;
		if (n==buffersize) {
			n=0;
			log(Warning) << "TracingContinous: set n to zero" << n <<"!" << endlog();
		}
	}
	
	fclose(pFile);
	
	this->stop();
	
	//string str = "rosrun rtt_control_components emaillogfile "+ filename + " " + to_string(buffer_nrports[BPID-1]) + " " + to_string(buffer_nrjoints[BPID-1]);
	//int result = system(str.c_str());
	
	return;
}

void TracingContinous::sendLog(int BPID)
{
	stopHook(BPID,n_cyclicbuffer);
}

ORO_CREATE_COMPONENT(Signal::TracingContinous)
