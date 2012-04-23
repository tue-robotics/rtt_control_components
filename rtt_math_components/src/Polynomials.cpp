/** Polynomials.cpp
 *
 * @class Polynomials
 *
 * \author Bas Willems
 * \date August, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Polynomials.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Polynomials::Polynomials(const string& name) : 
	    TaskContext(name, PreOperational)

{

  addProperty( "vector_size", vector_size ).doc("Size of the input vector.");

}

Polynomials::~Polynomials(){}

bool Polynomials::configureHook()
{
  Logger::In in("Polynomials::configureHook()");

  // Adding ports
  /* Guaranteeing Real-Time data flow */
  addEventPort( "in", inport );
  addPort( "out", outport );
  
  for ( uint i = 0; i < vector_size; i++ )
  {
    string name = "polynomial"+to_string(i+1);
    addProperty( name, polynomials[i]);
  }

  return true;
}

bool Polynomials::startHook()
{
  Logger::In in("Polynomials::startHook()");

  // Check validity of Ports:
  if ( !inport.connected() ) {
    log(Error)<<"inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }

  if ( !outport.connected() ) {
    log(Warning)<<"Outputport not connected!"<<endlog();
  }

  if ( vector_size < 1 ) {
    log(Error)<<"Polynomials parameters not valid!"<<endlog();
    return false;
  }

  return true;
}

void Polynomials::updateHook()
{
	// Read the input port
	doubles input(vector_size,0.0);
	doubles output(vector_size,0.0);
	
	// Read the input port
	inport.read( input );
	
	for (uint i = 0; i < vector_size; i++){
		int order=polynomials[i].size()-1;
		//printf("i=%d \n",i);
		for (int j = 0; j <=order; j++){
			
			output[i]+=polynomials[i][j]*pow(input[i],j);
			//printf("j=%d and adding %f to output which now = %f \n",j,polynomials[i][j]*pow(input[i],j-1),output[i]);
		}
	}
	
	// Write the outputs
	outport.write( output );
}	

ORO_CREATE_COMPONENT(MATH::Polynomials)
