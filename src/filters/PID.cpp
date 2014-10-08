/** PID.cpp
 *
 * @class PID
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date March, 2011
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "PID.hpp"

using namespace std;
using namespace RTT;
using namespace FILTERS;

PID::PID(const string& name) : 
    TaskContext(name, PreOperational),
    Ts(0.0), vector_size(0)
{

    addProperty( "proportional_coefficient", kp ).doc("Array of proportional coefficients of PID.");
    addProperty( "derivative_coefficient", kv ).doc("Array of derivative coefficients of PID.");
    addProperty( "integral_coefficient", ki ).doc("Array of integral coefficients of PID.");
    addProperty( "anti_windup_coefficient", kaw ).doc("Array of anti-windup coefficients of PID.");
    addProperty( "integrator_initial_value", init ).doc("Array of integrator initial values.");
    addProperty( "limit", limit ).doc("Array of saturation limits.");
    addProperty( "sampling_time", Ts).doc("Sampling time.");
    addProperty( "vector_size", vector_size ).doc("Size of the input vector.");

}

PID::~PID()
{
    for (unsigned int i = 0; i < vector_size; i++)
    {
        delete filters[i];
        filters[i] = 0;
    }
}

bool PID::configureHook()
{
    Logger::In in("PID::Configure");

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

        // Initialize filters, use Prewarp Tustin method for discretization
        filters[i] = new DFILTERS::DPID(kp[i], kv[i], ki[i], Ts, 4, limit[i], kaw[i]);
    }

    return true;
}

bool PID::startHook()
{
    Logger::In in("PID::Start");

    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"inputport not connected!"<<endlog();
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"Outputport not connected!"<<endlog();
    }

    if (Ts <= 0 || vector_size < 1 ) {
        log(Error)<<"PID parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (limit[i] < 0.0) {
            log(Error)<<"Limit can't be negative!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Warning)<<"PID: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void PID::updateHook()
{
    Logger::In in("PID::Update");

    // Read the input port
    doubles input(vector_size,0.0);
    doubles output(vector_size,0.0);

    // Read the input port
    inport.read( input );

    // Update filters
    for (uint i = 0; i < vector_size; i++) {
        filters[i]->update( input[i] );
        output[i] = filters[i]->getOutput();
    }

    // Integrator initial value
    // Do we need this?
    for (uint i = 0; i < vector_size; i++) {
        output[i] += init[i];
    }

    // Write the outputs
    outport.write( output );
}

ORO_CREATE_COMPONENT(FILTERS::PID)
