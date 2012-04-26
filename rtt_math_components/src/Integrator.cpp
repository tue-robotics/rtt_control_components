#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Integrator.hpp"

using namespace std;
using namespace RTT;
using namespace MATH;

Integrator::Integrator(const string& name) : TaskContext(name, PreOperational)
{
  /*** Default properties ***/
  N = 1;
  
  /*** Reading properties ***/
  addProperty( "vector_size", N );

  /*** Adding ports ***/
  addEventPort( "in", inport );
  addPort( "reset", resetport );
  addPort( "out", outport );
}
Integrator::~Integrator(){}

bool Integrator::configureHook()
{
  /* Guaranteeing Real-Time data flow */
  // create an example data sample of vector_size:
  doubles example(N, 0.0);

  // show it to the port (this is a not real-time operation):
  outport.setDataSample( example );
  /* Guaranteeing Real-Time data flow */

  previous_output.resize(N);

  return true;
}

bool Integrator::startHook()
{
  // Check validity of Ports:
  if ( !inport.connected() )
  {
    log(Error)<<"Integrator::inputport not connected!"<<endlog();
    // No connection was made, can't do my job !
    return false;
  }
  if ( !outport.connected() ) {
    log(Warning)<<"Integrator::Outputport not connected!"<<endlog();
  }

  //Start at value of initial port (will be zero if not connected).
  doubles init(N,0.0);
  if ( initialport.connected() )
  {
    initialport.read (init);
  }
  for ( uint i = 0; i < N; i++ )
  {
    previous_output[i] = init[i];
  }

  // Initialise old time:
  old_time = os::TimeService::Instance()->getNSecs()*1e-9;

  return true;
}

void Integrator::updateHook()
{
  // Read the inputport
  doubles input(N,0.0);
  doubles output(N,0.0);
  doubles reset(N,0.0);

  if ( resetport.read (reset) == NewData )
  {
	  for ( uint i = 0; i < N; i++ )
	    {
		  previous_output[i] = reset[i];
	    }
  }

  inport.read( input );

  double dt = determineDt();

  for ( uint i = 0; i < N; i++ )
  {
    output[i] = previous_output[i] + input[i] * dt;
    previous_output[i] = output[i];
  }

  // Write the outputs
  outport.write( output );
}

double Integrator::determineDt()
{
  long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  double dt = (double)(new_time - old_time);
  old_time = new_time;
  return dt;
}


ORO_CREATE_COMPONENT(MATH::Integrator)
