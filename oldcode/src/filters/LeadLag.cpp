/** LeadLag.cpp
*
* @class LeadLag
*
* \author Boris Mrkajic, Janno Lunenburg
* \date August, 2013
* \version 2.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "LeadLag.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

LeadLag::LeadLag(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0), Ts(0.0)
{

    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );
    addProperty( "zero_frequency", fz );
    addProperty( "pole_frequency", fp );

    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );
}

LeadLag::~LeadLag()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
        filters[i] = NULL;
    }
}

bool LeadLag::configureHook()
{

    filters.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {

        // Initialize filters, use Prewarp Tustin method for discretization
        filters[i] = new DFILTERS::DLeadLag(fz[i], fp[i], Ts, 4);
    }

    return true;
}

bool LeadLag::startHook()
{
		
    // Check validity of ports:
    if ( !inport.connected() ) {
        log(Error)<<"LeadLag::Input port not connected!"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"LeadLag::Output port not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"LeadLag parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fz[i] <= 0.0 || fp[i] <= 0.0){
            log(Error)<<"LeadLag parameters not valid!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Debug)<<"LeadLag: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void LeadLag::updateHook()
{
		
    // Read the input port
    doubles input(vector_size,0.0);
    doubles output(vector_size,0.0);

    inport.read( input );

    for (uint i = 0; i < vector_size; i++) {
        filters[i]->update( input[i] );
        output[i] = filters[i]->getOutput();
    }

    // Write the outputs
    outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::LeadLag)
