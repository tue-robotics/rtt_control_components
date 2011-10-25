/** DotProduct.cpp
 *
 * @class DotProduct
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "DotProduct.hpp"

#define multiply '*'
#define divide '/'

using namespace RTT;
using namespace MATH;

DotProduct::DotProduct(const string& name) : 
	TaskContext(name, PreOperational),
		list_of_operators("**"), vector_size(0)
{
  addProperty( "list_of_operators", list_of_operators ).doc("String that determines the number of input ports and specifies the input operators for the each of them. It can only consist of operators multiply and divide. e.g. ""**/*"" or ""*//"".");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the vector.");
}

DotProduct::~DotProduct(){}

bool DotProduct::configureHook()
{
	Logger::In in("DotProduct::configureHook()");
	N = list_of_operators.size();
	
	// Creating ports
	for ( uint i = 0; i < N; i++ )
	{
		string name_inport = "in"+to_string(i+1);
		addEventPort( name_inport, inports[i]);
	}
	addPort( "out", outport );

	return true;

}

bool DotProduct::startHook()
{
  Logger::In in("DotProduct::startHook()");

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
	  if (list_of_operators[j]!=multiply && list_of_operators[j]!=divide) {
		  log(Error)<<"list_of_operators can only consist of operators multiply and divide!"<<endlog();
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

void DotProduct::updateHook()
{
  doubles input(vector_size,0.0);
  doubles output(vector_size,1.0);

  for ( uint i = 0; i < N; i++ ) {
	  inports[i].read( input );
	  for ( uint j = 0; j < vector_size; j++ ) {
		  if (list_of_operators[i]==multiply) {
			  output[j] *= input[j];
		  }
		  if (list_of_operators[i]==divide) {
			  output[j] /= input[j];
		  }
	  }
  }

  /*** Write the outputs ***/
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::DotProduct)
