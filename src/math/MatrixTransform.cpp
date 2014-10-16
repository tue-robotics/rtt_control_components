#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "MatrixTransform.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

MatrixTransform::MatrixTransform(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "Ncolumns", Ncolumns );
	addProperty( "Nrows", Nrows );
}

MatrixTransform::~MatrixTransform(){}

bool MatrixTransform::configureHook()
{
	for ( uint i = 0; i < Nrows; i++ ) {
		string name = "function"+to_string(i+1);
		addProperty( name, function[i]);
	}

	// Creating ports:
	addEventPort( "in", inport );
	addPort( "out", outport );

	return true;
}

bool MatrixTransform::startHook()
{
	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"MatrixTransform::Inputport not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"MatrixTransform::Outputport not connected!"<<endlog();
	}
	return true;
}


void MatrixTransform::updateHook()
{
	// Read the inputports
	doubles input(Nrows,0.0);
	inport.read( input );

	// Do a matrix multiplication. Elementwise
	doubles output(Nrows,0.0);
	for ( uint i = 0; i < Nrows; i++ ) {
		output[i] = 0.0;
		for ( uint j = 0; j < Ncolumns; j++ ) {
			output[i] += function[i][j] * input[j];
		}
	}

	// Write the outputs
	outport.write( output );

}

ORO_CREATE_COMPONENT(MATH::MatrixTransform)
