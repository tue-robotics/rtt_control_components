/** Abs.cpp
*
* @class Abs
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "Abs.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Abs::Abs(const string& name) : 
	TaskContext(name, PreOperational),
		N(1), vector_size(0)
{
	addProperty( "number_of_inputs", N ).doc("An unsigned integer that determines the number of inputs. Default is 1");
	addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the vector.");
}

Abs::~Abs(){}

bool Abs::configureHook()
{
	Logger::In in("Abs::configureHook()");
	
	// Creating ports
	for ( uint i = 0; i < N; i++ )
	{
		string name_inport = "in"+to_string(i+1);
		string name_outport = "out"+to_string(i+1);
		addEventPort( name_inport, inports[i]);
		addPort( name_outport, outports[i] );
	}

	return true;
}

bool Abs::startHook()
{
  Logger::In in("Abs::startHook()");
  
 
  for (uint i = 0; i < N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i+1 <<" not connected!"<<endlog();
	  }
	  if ( !outports[i].connected() ) {
		log(Warning)<<"Output port "<< i+1 <<" not connected!"<<endlog();
		return false;
	  }
  } 
  
  if ( N > maxN ) {
    log(Error)<<"Max number of ports exceeded!"<<endlog();
    return false;
  }

  if ( N < 1 ) {
    log(Error)<<"Number of ports must be at least 1!"<<endlog();
    return false;
  }

  return true;
}

void Abs::updateHook()
{
  doubles input(vector_size,0.0);
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < N; i++ ) {
	  inports[i].read( input );
	  for ( uint j = 0; j < vector_size; j++ ) {
		  output[j] = fabs(input[j]);
	  }
	  outports[i].write( output );
  }

}

ORO_CREATE_COMPONENT(MATH::Abs)
