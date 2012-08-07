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
					buffersize(16384)
{
	addEventPort( "in", inport);
	addProperty( "vector_size", vectorsize ).doc("size of the vector (port)");
	addProperty( "buffersize", buffersize ).doc("Size of the buffer");
	addProperty( "filename", filename ).doc("Name of the file");

}

Tracing::~Tracing(){}

bool Tracing::configureHook()
{
	counter = -1;

	// TODO: Creat multiple ports
	columns = vectorsize;

	buffers.resize(columns); // Maybe create reserve()?
	for (uint column = 0; column < columns; column++)
	{
		buffers[column].resize(buffersize,-1);
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


	doubles input(columns,2.0);
	inport.read( input );

	// Fill it with data
	if(abs(counter) < buffersize)
	{

		for (uint column = 0; column < columns; ++column)
			{
				buffers[column][counter] = input[column];
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
	ofstream myFile (filename.c_str());

	std::vector<doubles>::iterator vit;
	doubles::iterator dit;
	for(vit = buffers.begin(); vit != buffers.end(); ++vit)
	{
		for(dit = (*vit).begin(); dit != (*vit).end(); ++dit)
		{
			myFile << *dit;
			myFile << "\n";
		}
	}



	myFile.close();

	fatal();

}

void Tracing::fatal()
{
	cout << "AAAAAAHHHHH!";
	vector<double> oeps;
	oeps[0] = oeps[1];
}

ORO_CREATE_COMPONENT(Tracing::Tracing)
