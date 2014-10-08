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
	addPort( "initial", initialport );
	addPort( "out", outport );
}
Integrator::~Integrator(){}

bool Integrator::configureHook()
{
	Logger::In in("Integrator::Configure");	
	
	/* Guaranteeing Real-Time data flow */
	doubles example(N, 0.0);
	// show it to the port (this is a not real-time operation):
	outport.setDataSample( example );

	previous_output.resize(N);

	return true;
}

bool Integrator::startHook()
{
	Logger::In in("Integrator::Start");		
	
	// Check validity of Ports:
	if ( !inport.connected() ) {
		log(Error)<<"Integrator::inputport not connected!"<<endlog();
		return false;
	}
	if ( !outport.connected() ) {
		log(Warning)<<"Integrator::Outputport not connected!"<<endlog();
	}

	//Start at value of initial port (will be zero if not connected).
	doubles init(N,0.0);
	if ( initialport.connected() ) {
		initialport.read (init);
	}
	for ( uint i = 0; i < N; i++ ) {
		previous_output[i] = init[i];
	}

	// Initialise old time:
	old_time = os::TimeService::Instance()->getNSecs()*1e-9;

	// Write the outputs to prevent sync issues
	outport.write( init );

	return true;
}

void Integrator::updateHook()
{
	Logger::In in("Integrator::Update");		
	
	doubles input(N,0.0);
	doubles output(N,0.0);
	doubles reset(N,0.0);

	if ( resetport.read (reset) == NewData ) {
		for ( uint i = 0; i < N; i++ ) {
			previous_output[i] = reset[i];
		}
	}
	
	// Read the inputport
	inport.read( input );

	double dt = determineDt();

	for ( uint i = 0; i < N; i++ ) {
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
