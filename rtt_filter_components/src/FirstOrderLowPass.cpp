/** FirstOrderLowPass.cpp
 *
 * @class FirstOrderLowPass
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "FirstOrderLowPass.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

FirstOrderLowPass::FirstOrderLowPass(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0), Ts(0.0)
{
    addProperty( "pole_frequency", fp );
    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );
}

FirstOrderLowPass::~FirstOrderLowPass()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
        filters[i] = NULL;
    }
}

bool FirstOrderLowPass::configureHook()
{
    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );

    /* Guaranteeing Real-Time data flow */
    // create an example data sample of vector_size:
    doubles example(vector_size, 0.0);

    // show it to the port (this is a not real-time operation):
    outport.setDataSample( example );
    /* Guaranteeing Real-Time data flow */

    filters.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {
        filters[i] = new DFILTERS::DFirstOrderLowpass(fp[i], Ts);
    }

    return true;
}

bool FirstOrderLowPass::startHook()
{
    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"FirstOrderLowPass::inputport not connected!"<<endlog();
        // No connection was made, can't do my job !
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"FirstOrderLowPass::Outputport not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"FirstOrderLowPass parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fp[i] <= 0.0){
            log(Error)<<"FirstOrderLowPass parameters not valid!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Warning)<<"FirstOrderLowPass: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void FirstOrderLowPass::updateHook()
{
    // Read the input port
    doubles input(vector_size,0.0);
    doubles output(vector_size,0.0);

    inport.read( input );

    // Update filters
    for (uint i = 0; i < vector_size; i++) {
        filters[i]->update(input[i]);
        output[i] = filters[i]->getOutput();
    }

    // Write the outputs
    outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::FirstOrderLowPass)
