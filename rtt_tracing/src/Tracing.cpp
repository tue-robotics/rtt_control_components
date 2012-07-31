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
					buffersize(16)
{
	addEventPort( "in", inport);
	addProperty( "buffersize", buffersize ).doc("Buffersize");
}

Tracing::~Tracing(){}

bool Tracing::configureHook()
{
	counter = 0;

	columns = 1;

	buffers.resize(columns); // Maybe create reserve()?
	for (int i = 0; i < columns; i++)
	{
		buffers[i].resize(buffersize,-1);
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

	doubles input(columns,2.0);
	inport.read( input );
	cout << input[0];
	// Fill it with data
	if(counter < buffersize)
	{
		//for(int column = 0; column != columns; ++column)
			buffers[0][0] = input.size();
		counter++;
	}
	else
	{
		// Display the data
		//std::vector<int>::iterator it;
		//for(it = buffers.begin(); it != buffers.end(); ++it)
		//	cout << *it;
		//for(index i = 0; i != 2; ++i)
		//	for(index j = 0; j != 2; ++j)
		//		buffers[i][j] = values++;
		stop();
	}


}

void Tracing::stopHook()
{
	ofstream myFile ("data.txt");
	myFile << "hello";

	vector<vector<double> > myVector(4,vector<double>(4,0.0)); //creates 4x4 nested vector
	vector<double> mycolumn=myVector[0];    //returns a reference to myVector[1], and assigns this vector to mycolumn
	double myValue=mycolumn[0];       //returns a reference to mycolumn[1], and assigns it to myValue

	//cout << buffers[1][1];

	myFile << buffers[0][0];
	myFile.close();
	/*
    int verify = 0;
    std::vector<doubles>::iterator it;
    		for(it = buffers.begin(); it != buffers.end(); ++it)
            assert(buffers[0][0] == verify++);
	 */


}

ORO_CREATE_COMPONENT(Tracing::Tracing)
