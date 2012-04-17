/** Sum.cpp
*
* @class Sum
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "Sum.hpp"

#define plus '+'
#define minus '-'

using namespace std;
using namespace RTT;
using namespace MATH;

Sum::Sum(const string& name) : 
	TaskContext(name, PreOperational),
		list_of_signs("++"), vector_size(0)
{
  addProperty( "list_of_signs", list_of_signs ).doc("String that determines the number of input ports and specifies the input sign for the each of them. It can only consist of pluses or minuses. e.g. ""+++-"" or ""+--"".");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the vector.");
}

Sum::~Sum(){}

bool Sum::configureHook()
{
	Logger::In in("Sum::configureHook()");
	N = list_of_signs.size();
	
	// Creating ports
	for ( uint i = 0; i < N; i++ )
	{
		string name_inport = "in"+to_string(i+1);
		addEventPort( name_inport, inports[i]);
	}
	addPort( "out", outport );

	return true;
}

bool Sum::startHook()
{
  Logger::In in("Sum::startHook()");
  
  for (uint i = 0; i < N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i+1 <<" not connected!"<<endlog();
	  }
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
    return false;
  }
  
  for ( uint j = 0; j < vector_size; j++ ) {
	  if (list_of_signs[j]!=plus && list_of_signs[j]!=minus) {
		  log(Error)<<"list_of_sings can only consist of pluses and minuses!"<<endlog();
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

void Sum::updateHook()
{
  Logger::In in("Sum::updateHook()");
  
  doubles input(vector_size,0.0);
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < N; i++ ) {
	  inports[i].read( input );
	  for ( uint j = 0; j < vector_size; j++ ) {
		  if (list_of_signs[i]==plus) {
			  output[j] += input[j];
		  }
		  if (list_of_signs[i]==minus) {
			  output[j] -= input[j];
		  }
	  }
  }
  
  outport.write( output );

}

ORO_CREATE_COMPONENT(MATH::Sum)
