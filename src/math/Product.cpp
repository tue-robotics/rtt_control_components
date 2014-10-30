/** Product.cpp
*
* @class Product
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Product.hpp"

#define multiply '*'
#define divide '/'

using namespace std;
using namespace RTT;
using namespace MATH;

Product::Product(const string& name) : 
	TaskContext(name, PreOperational),
		list_of_operators("**")
{

  addProperty( "list_of_operators", list_of_operators );

}

Product::~Product(){}

bool Product::configureHook()
{
	Logger::In in("Product::configureHook()");
	Ts = getPeriod();	
	N = list_of_operators.size();
	
	inports.resize(N);
	
	// Adding ports
	for (uint i = 0; i < N; i++) {
		std::string portname("in");
		std::ostringstream os;
		os << (i+1);
		portname += os.str();
		addPort( portname, inports[i]);
	}
	addPort( "out", outport );   
  
	return true;
}

bool Product::startHook()
{
  Logger::In in("Product::startHook()");

  // Check validity of Ports:
  for (uint i = 0; i < N; i++) {
	  if ( !inports[i].connected() ) {
		  log(Error)<<"Input port "<< i <<" not connected!"<<endlog();
		  // No connection was made, can't do my job !
		  return false;
	  }
  }
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }

  if (N < 1 || Ts <= 0) {
    log(Error)<<"Product parameters not valid!"<<endlog();
    return false;
  }
  
  for ( uint i = 0; i < N; i++ ) {
	 if (list_of_operators[i]!=multiply && list_of_operators[i]!=divide) {
		 log(Error)<<"Product parameters not valid!"<<endlog();
		 return false;
	 }
  }

  return true;
}

void Product::updateHook()
{
  /*** Calculate outputs ***/
  double input = 0.0;
  double output = 1.0;
  for ( uint i = 0; i < N; i++ ) {
	  inports[i].read( input );
	  if (list_of_operators[i]==multiply) {
		  output *= input;
	  }
	  if (list_of_operators[i]==divide) {
		  output /= input;
	  }
  }
  
  /*** Write the outputs ***/
  outport.write( output );
}

ORO_CREATE_COMPONENT(MATH::Product)
