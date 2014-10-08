/** SkewedNotch.cpp
*
* @class SkewedNotch
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SkewedNotch.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

SkewedNotch::SkewedNotch(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0),Ts(0.0)
{

    addProperty( "zero_frequency", fz );
    addProperty( "zero_damping", dz );
    addProperty( "pole_frequency", fp );
    addProperty( "pole_damping", dp );
    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );

    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );
}

SkewedNotch::~SkewedNotch()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
    }
}

bool SkewedNotch::configureHook()
{
 	Logger::In in("SkewedNotch::Configure");
 	
    /* Guaranteeing Real-Time data flow */
    // create an example data sample of vector_size:
    doubles example(vector_size, 0.0);

    // show it to the port (this is a not real-time operation):
    outport.setDataSample( example );
    /* Guaranteeing Real-Time data flow */

    filters.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {

        // Default discretization method: Prewarp Tustin
        filters[i] = new DFILTERS::DSkewedNotch(fz[i], dz[i], fp[i], dp[i], Ts, 4);
    }

    return true;
}

bool SkewedNotch::startHook()
{ 	
	Logger::In in("SkewedNotch::Start");
    
    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"inputport not connected!"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"Outputport not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"SkewedNotch parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fz[i] <= 0.0 || dz[i] <= 0.0 || fp[i] <= 0.0 || dp[i] <= 0.0) {
            log(Error)<<"SkewedNotch parameters not valid!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Warning)<<"SkewedNotch: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void SkewedNotch::updateHook()
{
	Logger::In in("SkewedNotch::Update");
	
    // Read the input port
    doubles input(vector_size,0.0);
    doubles output(vector_size,0.0);

    inport.read( input );

    // Update filters
    for (uint i = 0; i < vector_size; i++) {
        filters[i]->update( input[i] );
        output[i] = filters[i]->getOutput();
    }

    // Write the outputs
    outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::SkewedNotch)
