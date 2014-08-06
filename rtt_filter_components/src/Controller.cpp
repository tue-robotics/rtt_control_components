/** Controller.hpp
 *
 * @class Controller
 *
 * \author Max Baeten
 * \date August, 2014
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "Controller.hpp"

using namespace std;
using namespace RTT;
using namespace FILTERS;

Controller::Controller(const string& name) : 
    TaskContext(name, PreOperational),
    vector_size(0)
{
    addProperty( "vector_size", vector_size );
    addProperty( "gains", gains );

    // Adding ports
    addEventPort( "in", inport_references );
    addEventPort( "in", inport_positions );
    addPort( "out", outport_controloutput );
}

Controller::~Controller(){}

bool Controller::configureHook()
{
    return true;
}

bool Controller::startHook()
{
    // Check validity of Ports:
    if ( !inport_references.connected() || !inport_positions.connected() ) {
        log(Error)<<"Controller: One of the inports is not connected!"<<endlog();
        return false;
    }

    if ( !outport_controloutput.connected() ) {
        log(Error)<<"Controller: Outputport not connected!"<<endlog();
        return false;
    }

    return true;
}

void Controller::updateHook()
{
    doubles references(vector_size,0.0);
    doubles positions(vector_size,0.0);
    doubles controloutput(vector_size,0.0);
    doubles controlerrors(vector_size,0.0);

    // Read the input ports
    inport_references.read(references);
    inport_positions.read(positions);

    // Compute joint errors
    for (uint i = 0; i < vector_size; i++) {
        controlerrors[i] = references[i]-positions[i];
    }

    // Compute control output
    for (uint i = 0; i < vector_size; i++) {
        controloutput[i] = controlerrors[i]*gains[i];
    }

    // Write the outputs
    outport_controloutput.write( controloutput );

}

ORO_CREATE_COMPONENT(FILTERS::Controller)
