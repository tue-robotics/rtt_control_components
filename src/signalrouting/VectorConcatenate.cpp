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
	TaskContext(name, PreOperational)
{
  addProperty( "vector_size", N ).doc("An unsigned integer that specifies the number of input ports");
  addProperty( "event_port",  EventPorts ).doc("An array that specifies for each input wether it should be an eventPort. 1.0 for Event, 0.0 for non Event port");
}

VectorConcatenate::~VectorConcatenate(){}

bool VectorConcatenate::configureHook()
{
	if (N > maxN) {
		log(Error)<<"VectorConcatenate: N is larger than maxN, more than " << maxN << " components are not supported. Change maxN in the .hpp" <<endlog();
		return false;
	}	
	if (EventPorts.size() != N) {
		log(Error)<<"VectorConcatenate: event_port size does not match vector_size"<<endlog();
		return false;
	}
	
	for ( uint i = 0; i < N; i++ )
	{
		if (EventPorts[i] != 0.0) {
			addEventPort( ("in"+to_string(i+1)), inports[i] );
		}
		else {
			addPort( ("in"+to_string(i+1)), inports[i] );			
		}
	}

	addPort( "out", outport );
	
	return true;
}

bool VectorConcatenate::startHook()
{  
  for (uint i = 0; i < N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i <<" not connected!"<<endlog();
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

  outport.write( output );
}

ORO_CREATE_COMPONENT(SIGNALROUTING::VectorConcatenate)
