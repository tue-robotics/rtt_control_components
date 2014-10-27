/** SecondOrderLowPasses.cpp
*
* @class SecondOrderLowPasses
*
* \author Boris Mrkajic, Janno Lunenburg
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SecondOrderLowPass.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

SecondOrderLowPasses::SecondOrderLowPasses(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0), Ts(0.0)
{

    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );
    addProperty( "pole_frequency", fp );
    addProperty( "pole_damping", dp );

    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );
}

SecondOrderLowPasses::~SecondOrderLowPasses()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
    }
}

bool SecondOrderLowPasses::configureHook()
{

    filters.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {

        // Initialize filters, use Prewarp Tustin method for discretization
        filters[i] = new DFILTERS::DSecondOrderLowpass(fp[i], dp[i], Ts, 4);
    }

    return true;
}

bool SecondOrderLowPasses::startHook()
{
	
    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"SecondOrderLowPasses::inputport not connected!"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"SecondOrderLowPasses::Outputport not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"SecondOrderLowPass parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fp[i] <= 0.0 || dp[i] <= 0.0){
            log(Error)<<"SecondOrderLowPass parameters not valid!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Debug)<<"SecondOrderLowPasses: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void SecondOrderLowPasses::updateHook()
{
	
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

ORO_CREATE_COMPONENT(FILTERS::SecondOrderLowPasses)
