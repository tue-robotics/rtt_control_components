/** GaussianRandomNumber.cpp
*
* @class GaussianRandomNumber
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/
 
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <ocl/Component.hpp>

#include "GaussianRandomNumber.hpp"

using namespace RTT;
using namespace SOURCES;

GaussianRandomNumber::GaussianRandomNumber(const string& name) : 
	TaskContext(name, PreOperational),
		vector_size(0)
{

  /*** Reading properties ***/
  addProperty( "mean", mean ).doc("An array of double values that determines the mean value of the generated signals.");
  addProperty( "variance", variance ).doc("An array of double values that determines the variance of the generated signals.");
  addProperty( "vector_size", vector_size ).doc("An unsigned integer that specifies the size of the output vector.");

  /*** Adding ports ***/
  addPort( "out", outport ).doc("Vector of double values");
  
}

GaussianRandomNumber::~GaussianRandomNumber(){}

bool GaussianRandomNumber::configureHook()
{
  Logger::In in("GaussianRandomNumber::configureHook()");
   
  Ts = getPeriod();
  return true;
}

bool GaussianRandomNumber::startHook()
{
  Logger::In in("GaussianRandomNumber::startHook()");
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }
  
  if (vector_size < 1) {
    log(Error)<<"GaussianRandomNumber parameters not valid!"<<endlog();
    return false;
  }

  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }

  for ( uint i = 0; i < vector_size; i++ ) {
	  if (variance[i] < 0.0){
		  log(Error)<<"Variance not valid!"<<endlog();
	      return false;
	  }
  }

  return true;
}

void GaussianRandomNumber::updateHook()
{
  doubles output(vector_size,0.0);
  
  for ( uint i = 0; i < vector_size; i++ ) {
	  output[i] = box_muller(mean[i], variance[i]);
  }
   
  /*** Write the outputs ***/
  outport.write( output );
}

double GaussianRandomNumber::randomnumgen(double low, double high)
/* uniform random variate generator */
/* minimum low, maximum high */
{
  double range=(high-low);
  double num = rand() * range / RAND_MAX + low ;
  return(num);
}

double GaussianRandomNumber::box_muller(double m, double v)	
/* normal random variate generator */
/* mean m, variance v */
{								
	double x1, x2, w, y1;
	static double y2;
	static int use_last = 0;

	/* use value from previous call */
	if (use_last)		       
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * randomnumgen(0.0,1.0) - 1.0;
			x2 = 2.0 * randomnumgen(0.0,1.0) - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( m + y1 * sqrt(v) );
}

ORO_CREATE_COMPONENT(SOURCES::GaussianRandomNumber)
