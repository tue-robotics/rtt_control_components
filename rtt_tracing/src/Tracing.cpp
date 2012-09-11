/** Tracing.cpp
 *
 * @class Tracing
 *
 * \author Tim Clephas
 * \date Summer, 2012
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <fstream>


#include "Tracing.hpp"

using namespace std;
using namespace RTT;
using namespace Signal;

Tracing::Tracing(const string& name) : 
					TaskContext(name, PreOperational),
					filename("data.txt"),
					buffersize(16384),
					Ts(0.001),
					crash(true)
{
	addProperty( "vector_sizes", vectorsizes_prop ).doc("size of the vector per port. Example: array ( 2.0 4.0 4.0 )");
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
	addProperty( "Crash_if_done", crash ).doc("Let orocos crash once the logging is finished");
}

Tracing::~Tracing(){}

bool Tracing::configureHook()
{

	// Create ports based on the number of vector sizes given
	uint Nports = vectorsizes_prop.size();
	vectorsizes.resize(Nports);

	// Creating ports
	for ( uint i = 0; i < Nports; i++ )
	{
		vectorsizes[i] = vectorsizes_prop[i]; // Hack
		string name_inport = "in"+to_string(i+1);
		addEventPort( name_inport, inports[i]);
		columns += vectorsizes[i];
	}

	counter = -1;

	columns += 1 ; // for time vector

	cout << columns;
	stop();
	buffers.resize(buffersize); // Maybe create reserve()?
	for (uint line = 0; line < buffersize; line++)
	{
		buffers[line].resize(columns,-1);
	}


	return true;
}

bool Tracing::startHook()
{
	/*if ( !inports[0].connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}*/
	return true;
}

void Tracing::updateHook()
{
	// First updatehook is useless
	if(counter == -1){counter = 0;return;}

	doubles input(vectorsizes[0],-2.0);
	inports[0].read( input );

	// Fill it with data
	if(abs(counter) < buffersize)
	{
		buffers[counter][0] = Ts*counter;
		for (uint column = 1; column < columns; ++column)
			{
				buffers[counter][column] = input[column-1];
			}

		counter++;
	}
	else
	{
		stop();
	}
}

void Tracing::stopHook()
{
	FILE * pFile;
	pFile = fopen (filename.c_str(),"w");

	string portname = inports[0].getName();
	fprintf(pFile, "Time\t%s\n", portname.c_str());

	std::vector<doubles>::iterator vit;
	doubles::iterator dit;
	for(vit = buffers.begin(); vit != buffers.end(); ++vit)
	{
		for(dit = (*vit).begin(); dit != (*vit).end(); ++dit)
		{
			fprintf(pFile, "% .6e\t", *dit);
		}
		fprintf(pFile, "\n");
	}


	fclose(pFile);

	if ( crash )
		fatal();

}

void Tracing::fatal()
{
	cout << "AAAAAAHHHHH!";
	vector<double> oeps;
	oeps[0] = oeps[1];
}

ORO_CREATE_COMPONENT(Signal::Tracing)
