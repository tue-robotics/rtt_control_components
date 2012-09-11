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
	addEventPort( "in", inport);
	addProperty( "vector_size", vectorsize ).doc("size of the vector (port)");
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");
	addProperty( "Ts", Ts ).doc("Sample time of orocos, used for time vector");
	addProperty( "Crash_if_done", crash ).doc("Let orocos crash once the logging is finished");
}

Tracing::~Tracing(){}

bool Tracing::configureHook()
{
	counter = -1;

	// TODO: Creat multiple ports
	columns = vectorsize +1 ; //+1 for time vector

	buffers.resize(buffersize); // Maybe create reserve()?
	for (uint line = 0; line < buffersize; line++)
	{
		buffers[line].resize(columns,-1);
	}


	return true;
}

bool Tracing::startHook()
{
	if ( !inport.connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}
	return true;
}

void Tracing::updateHook()
{
	// First updatehook is useless
	if(counter == -1){counter = 0;return;}

	doubles input(vectorsize,-2.0);
	inport.read( input );

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

	string portname = inport.getName();
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
