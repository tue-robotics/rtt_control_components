/** WeakIntegrators.cpp
*
* @class WeakIntegrators
*
* \author Boris Mrkajic, Janno Lunenburg
* \date August, 2013
* \version 2.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "WeakIntegrators.hpp"

#define PI 3.14159265358979
#define eps 1e-16

using namespace std;
using namespace RTT;
using namespace FILTERS;

WeakIntegrators::WeakIntegrators(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0),Ts(0.0)
{

    addProperty( "zero_frequency", fz );
    addProperty( "vector_size", vector_size );
    addProperty( "sampling_time", Ts );

    // Adding ports
    addEventPort( "in", inport );
    addPort( "out", outport );
}

WeakIntegrators::~WeakIntegrators()
{
	for (unsigned int i = 0; i < vector_size; i++) {
		delete filters[i];
		filters[i] = NULL;
	}
}
	

bool WeakIntegrators::configureHook()
{
    /* Guaranteeing Real-Time data flow */
    // create an example data sample of vector_size:
    doubles example(vector_size, 0.0);

    // show it to the port (this is a not real-time operation):
    outport.setDataSample( example );
    /* Guaranteeing Real-Time data flow */

    filters.resize(vector_size);
    for (uint i = 0; i < vector_size; i++) {

        // Default discretization method: Prewarp Tustin
        if (fz[i] != 0.0)
        {
			filters[i] = new DFILTERS::DWeakIntegrator(fz[i], Ts, 4);
		}
		else
		{
			// If zero frequency equals zero, default construction leads to filter with gain 1, slope 0
			filters[i] = new DFILTERS::DWeakIntegrator();
		}
    }

    return true;
}

bool WeakIntegrators::startHook()
{
    // Check validity of Ports:
    if ( !inport.connected() ) {
        log(Error)<<"WeakIntegrators::inputport not connected!"<<endlog();
        // No connection was made, can't do my job !
        return false;
    }

    if ( !outport.connected() ) {
        log(Warning)<<"WeakIntegrators::Outputport not connected!"<<endlog();
    }

    if (vector_size < 1 || Ts <= 0.0) {
        log(Error)<<"WeakIntegrators:: parameters not valid!"<<endlog();
        return false;
    }

    for (uint i = 0; i < vector_size; i++) {
        if (fz[i] < 0.0) {
            log(Error)<<"WeakIntegrators:: parameters not valid!"<<endlog();
            return false;
        }
    }

    /// Print debug info
    for (uint i = 0; i < vector_size; i++)
    {
        log(Debug)<<"WeakIntegrator: output[i] = "<<filters[i]->getOutput()<<endlog();
    }

    return true;
}

void WeakIntegrators::updateHook()
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

ORO_CREATE_COMPONENT(FILTERS::WeakIntegrators)
