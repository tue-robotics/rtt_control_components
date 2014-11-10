/** SmallerLarger.cpp
 *
 * @class SmallerLarger
 *
 * \author Ton Peters
 * \date October, 2014
 * \version 1.0
 *
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "SmallerLarger.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

SmallerLarger::SmallerLarger(const string& name) :
    TaskContext(name, PreOperational),
    direct_to_ROS(false)
{
    // Adding ports
    addProperty( "numberofinports", n_signals ).doc("The number of inports");
    addProperty( "numberofoutports", n_outports ).doc("The number of outports");
    addProperty( "input_sizes", input_sizes ).doc("Vector specifying sizes of the inports");
    addProperty( "output_sizes", output_sizes ).doc("Vector specifying sizes of the outports");
    addProperty( "bound_values", bound_values ).doc("Vector specifying the bounding values for comparison");
    addProperty( "smaller", smaller ).doc("Vector of bools, specifying true means smaller, false means larger");
    addProperty( "direct_to_ROS", direct_to_ROS ).doc("Boolean specifying wether the output has to be streamed directly to ROS");
}

SmallerLarger::~SmallerLarger(){}

bool SmallerLarger::configureHook()
{
    // Determine number of in- and outputs
    n_signals = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        n_signals += input_sizes[i];
    }
    outputs.assign(n_signals,false);

    // Verify property feasibility
    if (input_sizes.size()  == 0 || output_sizes.size()  == 0 ) {
        log(Error) << "SmallerLarger: Please make sure the input_sizes and output_sizes are properly set before the call to the configureHook()" << endlog();
        return false;
    }
    if (n_inports  > maxN || n_outports  > maxN ) {
        log(Error) << "SmallerLarger: The maximum number of ports has exceeded the hardcoded value maxN. Verify the number of ports and if necessary increase maxN" << endlog();
        return false;
    }
    if (input_sizes.size()  != n_inports || output_sizes.size()  != n_outports ) {
        log(Error) << "SmallerLarger: The size of input_sizes/output_sizes does not match the corresponding numberofinports/numberofoutports " << endlog();
        return false;
    }
    if ( bound_values.size() != n_signals) {
        log(Error) << "SmallerLarger: The size of bound_values does not match n_inputs (which is the sum of input_sizes)" << endlog();
        return false;
    }
    if ( smaller.size() != n_signals) {
        log(Error) << "SmallerLarger: The size of smaller_larger does not match n_inputs (which is the sum of input_sizes)" << endlog();
        return false;
    }
    if (direct_to_ROS) {
        for ( uint i = 0; i < n_outports; i++ ) {
            if (output_sizes[i] != 1.0) {
                log(Error) << "SmallerLarger: direct_to_ROS is true, But one of the output sizes is not equal to 1.0" << endlog();
                return false;
            }
        }
    }

    // Resizing of inputdata, and outputdata
    inputdata.resize(n_inports);
    for ( uint i = 0; i < n_inports; i++ ) {
        inputdata[i].resize(input_sizes[i]);
    }
    outputdata.resize(n_outports);
    for ( uint i = 0; i < n_outports; i++ ) {
        outputdata[i].resize(output_sizes[i]);
    }

    // Creating in- and outports
    for ( uint i = 0; i < n_inports; i++ ) {
        string name_inport = "in"+to_string(i+1);
        addEventPort( name_inport, inports[i] );
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        string name_outport = "out"+to_string(i+1);
        addPort( name_outport, outports[i] );
        string name_outport_toROS = "outmsg"+to_string(i+1);
        addPort( name_outport_toROS, outports_toROS[i] );
    }

    return true;
}

bool SmallerLarger::startHook()
{

    return true;
}

void SmallerLarger::updateHook()
{
    // Check all ports for newdata and calculate output
    uint column_iterator = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        if ( NewData == inports[i].read(inputdata[i])) {
            for (uint j = 0; j<input_sizes[i]; j++) {
                if ( smaller[column_iterator+j] ){
                    outputs[column_iterator+j] = (inputdata[i][j] < bound_values[column_iterator+j]);
                } else {
                    outputs[column_iterator+j] = (inputdata[i][j] > bound_values[column_iterator+j]);
                }
            }
        }
        column_iterator += input_sizes[i];
    }

    column_iterator = 0;
    for ( uint i=0; i < n_outports; i++ ){
        for( uint j=0; j<output_sizes[i]; j++){
            outputdata[i][j] = outputs[column_iterator+j];
        }
        column_iterator += output_sizes[i];
    }

    if (!direct_to_ROS) {
        // Write output
        for ( uint i = 0; i < n_outports; i++ ) {
            outports[i].write(outputdata[i]);
        }
    }
    else {
        // Convert output to message and write msg
        for ( uint i = 0; i < n_outports; i++ ) {
            outputdata_msg.data = outputdata[i][0];
            outports_toROS[i].write(outputdata_msg);
        }
    }
}

ORO_CREATE_COMPONENT(ROS::SmallerLarger)
