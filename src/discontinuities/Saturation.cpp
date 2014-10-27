/** Saturation.cpp
*
* @class Saturation
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Saturation.hpp"

using namespace std;
using namespace RTT;
using namespace DISCONTINUITIES;

Saturation::Saturation(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{

  addProperty( "upper_limit", upper_limit );
  addProperty( "lower_limit", lower_limit );
  addProperty( "vector_size", vector_size );
  
  addEventPort( "in", inport);
  addPort( "out", outport); 
   
}

Saturation::~Saturation(){}

bool Saturation::configureHook()
{

   return true;
}

bool Saturation::startHook()
{

	if ( !inport.connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}

	if ( !outport.connected() ) {
		log(Warning)<<"Output port not connected!"<<endlog();
	}

	for (uint i = 0; i < vector_size; i++) {
		if (upper_limit[i] < lower_limit[i]) {
			log(Error)<<"Parameters not valid!"<<endlog();
			return false;
		}
	}

	return true;
}

void Saturation::updateHook()
{

	doubles values(vector_size,0.0);

	inport.read( values );

	for (uint i = 0; i < vector_size; i++) {
		if (values[i] > upper_limit[i]) {
			values[i] = upper_limit[i];
		}
		if (values[i] < lower_limit[i]) {
			values[i] = lower_limit[i];
		}
	}

	outport.write( values );
}

ORO_CREATE_COMPONENT(DISCONTINUITIES::Saturation)
