/** VectorConcatenate.cpp
*
* @class VectorConcatenate
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "VectorConcatenate.hpp"

using namespace std;
using namespace RTT;
using namespace SIGNALROUTING;

VectorConcatenate::VectorConcatenate(const string& name) : 
	TaskContext(name, PreOperational),
		N(2)
{
  addProperty( "number_of_inputs", N ).doc("An unsigned integer that specifies the number of input ports");;
}

VectorConcatenate::~VectorConcatenate(){}

bool VectorConcatenate::configureHook()
{
	Logger::In in("VectorConcatenate::configureHook()");
	
	//inports.resize(N);

	/*
	// Adding ports
	for (uint i = 0; i < N; i++) {
		std::string portname("in");
		std::ostringstream os;
		os << (i+1);
		portname += os.str();
		addPort( portname, inports[i] );
	}*/
	
	for ( uint i = 0; i < N-1; i++ )
	{
		string name_inport = "in"+to_string(i+1);
		addEventPort( name_inport, inports[i] );
	}
	string name_inport = "in"+to_string(N)+"_event";
	addEventPort( name_inport, inports[N-1] );
	
	addPort( "out", outport );
	
	return true;
}

bool VectorConcatenate::startHook()
{
  Logger::In in("VectorConcatenate::startHook()");
  
  // Check validity of Ports:
  for (uint i = 0; i < N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i <<" not connected!"<<endlog();
		  // No connection was made, can't do my job !
		  return false;
	  }
  }
  
  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }
  
  if (N < 1 ) {
    log(Error)<<"VectorConcatenate parameters not valid!"<<endlog();
    return false;
  }

  return true;
}

void VectorConcatenate::updateHook()
{
  Logger::In in("VectorConcatenate::updateHook()");

  doubles output;  
  double vector_size = 0;

  for ( uint i = 0; i < N; i++ ) {
	  doubles input;
	  inports[i].read( input );
	  vector_size = input.size();
	  for ( uint j = 0; j < vector_size; j++ ) {
		  output.push_back(input[j]);
	  }
  }

  // Write the outputs
  outport.write( output );
}

ORO_CREATE_COMPONENT(SIGNALROUTING::VectorConcatenate)
