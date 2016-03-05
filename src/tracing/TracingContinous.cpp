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
	addProperty( "vector_sizes", vectorsizes_prop ).doc("size of the vector per port. Example: array ( 2.0 4.0 4.0 )");
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
}

TracingContinous::~TracingContinous(){}

bool TracingContinous::configureHook()
{
	
	columns = 0;

	// Create ports based on the number of vector sizes given
	Nports = vectorsizes_prop.size();
	vectorsizes.resize(Nports);

	// Creating ports
	for ( uint i = 0; i < Nports; i++ )
	{
		vectorsizes[i] = vectorsizes_prop[i]; // Hack
		if (i == 0) {
			string name_inport = "in"+to_string(i+1);
			addEventPort( name_inport, inports[i]);
		}
		else {
			string name_inport = "in"+to_string(i+1);
			addPort( name_inport, inports[i]);
		}
		columns += vectorsizes[i];
		cout << "Column size: ";
		cout << columns;
		cout << "\n";
	}

	counter = -1;

	buffers.resize(buffersize); // Maybe create reserve()?
	for (uint line = 0; line < buffersize; line++)
	{
		buffers[line].resize(columns,-1);
	}


	return true;
}

bool TracingContinous::startHook()
{
	
	printed = false;
	
	return true;
}

void TracingContinous::updateHook()
{
	
	// First updatehook is useless
	if(counter == -1){counter = 0;return;}
	// TODO: if ( !this->isRunning() ) return; (Check if this also works, counter can become a uint
	
	if (counter == 10 && printed == false) {
		log(Warning) << "TracingContinous: Started tracing!" << endlog();
		printed = true;
		}
	
	uint startcolumn = 0;
	for ( uint i = 0; i < Nports; i++ )
	{
		doubles input(vectorsizes[i],-2.0);

		if ( inports[i].read( input ) == NewData )
		{
			uint inputiterator = 0;
			// Fill it with data
			for (uint column =  startcolumn; column < startcolumn + vectorsizes[i]; ++column)
			{
				buffers[counter][column] = input[inputiterator];
				inputiterator++;
			}
		}
		else {
			// Fill it with -1 since no data seen
			for (uint column =  startcolumn; column < startcolumn + vectorsizes[i]; ++column)
			{
				buffers[counter][column] = -1;
			}
			//if (i==1) log(Warning)<<"tracing:: no new data recieved on event port"<<endlog();
		}
		startcolumn += vectorsizes[i];
	}

	counter++;

	// Stop if buffer is full
	if(abs(counter) >= buffersize) 
	{
		log(Warning) << "TracingContinous: Trace stop!" << endlog();
		stop();
	}
}

void TracingContinous::stopHook()
{	
	FILE * pFile;
	pFile = fopen (filename.c_str(),"w");


	fprintf(pFile, "Time    \t");
	for ( uint i = 0; i < vectorsizes.size(); i++ )
	{
		string portname = inports[i].getName();
		for ( int j = 0; j < vectorsizes[i]; j++ )
		{
			fprintf(pFile, " %s[%d]    \t", portname.c_str(), j );
			//char columnheader = "["+to_string(j)+"]\t";
			//portname.c_str()+
			//fprintf(pFile, "%s\t", columnheader);
			//cout << columnheader;
		}
	}
	fprintf(pFile, "\n");
	//

	uint line = 0;
	std::vector<doubles>::iterator vit;
	doubles::iterator dit;
	for(vit = buffers.begin(); vit != buffers.end(); ++vit)
	{
		double timevalue = line*Ts;
		fprintf(pFile, "%f\t", timevalue);
		for(dit = (*vit).begin(); dit != (*vit).end(); ++dit)
		{
			fprintf(pFile, "% .6e\t", *dit);
			//fprintf(pFile, "% f\t", *dit);
		}
		fprintf(pFile, "\n");
		line++;
	}

	log(Warning) << "TracingContinous: Trace written!" << endlog();

	fclose(pFile);

}

void TracingContinous::Send_Email(char * msgbuf)
{
	char tmpfile[100];
	char command[2000];
	FILE * fp;

	memset(tmpfile,0x00,sizeof(tmpfile));
	memset(command,0x00,sizeof(command));

	strcpy(tmpfile,mkstemp("/tmp",NULL));

	if ( (fp=fopen(tmpfile,"w"))) {
		fprintf(fp,"APDU Process %d has an error.\n", getpid());
		fprintf(fp,"\n%s",msgbuf);

		fclose(fp);

		sprintf(command,"cat %s | sendmail -s \"SOME SUBJECT\" \"some_email@some_pace.com me@myself.com\"", tmpfile);

		int sc_returnvalue;
		sc_returnvalue = system(command);
		remove(tmpfile);
	}

}

ORO_CREATE_COMPONENT(Signal::TracingContinous)    

