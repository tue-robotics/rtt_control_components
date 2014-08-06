#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogInsGeneric.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogInsGeneric::AnalogInsGeneric(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "numberofinports", n_inports ).doc("The number of inports");
    addProperty( "numberofoutports", n_outports ).doc("The number of outports");
    addProperty( "input_sizes", input_sizes ).doc("Vector specifying sizes of the inports");
    addProperty( "output_sizes", output_sizes ).doc("Vector specifying sizes of the outports");
}
AnalogInsGeneric::~AnalogInsGeneric(){}

bool AnalogInsGeneric::configureHook()
{
    // Verify if supplied properties are feasible
    if (input_sizes.size()  == 0 || output_sizes.size()  == 0 ) {
        log(Error) << "AnalogInsGeneric: Please make sure the input_sizes and output_sizes are properly set before the call to the configureHook()" << endlog();
        return false;
    }
    if (n_inports  > maxN || n_outports  > maxN ) {
        log(Error) << "AnalogInsGeneric: The maximum number of ports has exceeded the hardcoded value maxN. Verify the number of ports and if necessary increase maxN" << endlog();
        return false;
    }
    if (input_sizes.size()  != n_inports || output_sizes.size()  != n_outports ) {
        log(Error) << "AnalogInsGeneric: The size of input_sizes/output_sizes does not match the corresponding numberofinports/numberofoutports " << endlog();
        return false;
    }

    // Determine number of in- and outputs
    n_inputs = 0;
    n_outputs = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        n_inputs += input_sizes[i];
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        n_outputs += output_sizes[i];
    }

    // Resizing of inputdata_msgs, and outputdata
    inputdata_msgs.resize(n_inports);
    for ( uint i = 0; i < n_inports; i++ ) {
        inputdata_msgs[i].values.resize(input_sizes[i]);
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
    }

    // Create mapping matrix
    mapping.resize(max(n_inputs,n_outputs));
    for ( uint i = 0; i < max(n_inputs,n_outputs); i++ ) {
        mapping[i].assign(4,0.0);
    }

    uint k = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        for ( uint j = 0; j < input_sizes[i]; j++ ) {
            if (k < min(n_inputs,n_outputs)) {
                mapping[k][0] = i;
                mapping[k][1] = j;
            }
            k++;
        }
    }

    k = 0;
    for ( uint i = 0; i < n_outports; i++ ) {
        for ( uint j = 0; j < output_sizes[i]; j++ ) {
            if (k < min(n_inputs,n_outputs)) {
                mapping[k][2] = i;
                mapping[k][3] = j;
            }
            k++;
        }
    }

    return true;
}

bool AnalogInsGeneric::startHook()
{
    // Connection checks
    for ( uint i = 0; i < n_inports; i++ ) {
        if ( !inports[i].connected() ) {
          log(Warning)<<"AnalogInsGeneric:: in"<< i+1 <<" not connected!"<<endlog();
        }
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        if ( !outports[i].connected() ) {
          log(Warning)<<"AnalogInsGeneric:: out"<< i+1 <<" not connected!"<<endlog();
        }
    }

	return true;
}

void AnalogInsGeneric::updateHook()
{
    // Check all ports for newdata
    for ( uint i = 0; i < n_inports; i++ ) {
        if ( NewData == inports[i].read(inputdata_msgs[i])) { }
    }

    // Do map input structure to output structure
    uint k = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        for ( uint j = 0; j < input_sizes[i]; j++ ) {
            uint map_in_1 = mapping[k][0];
            uint map_in_2 = mapping[k][1];
            uint map_out_1 = mapping[k][2];
            uint map_out_2 = mapping[k][3];
            outputdata[map_out_1][map_out_2] = inputdata_msgs[map_in_1].values[map_in_2];
            k++;
        }
    }

    // Write Output
    for ( uint i = 0; i < n_outports; i++ ) {
        outports[i].write(outputdata[i]);
    }
}

ORO_CREATE_COMPONENT(SOEM::AnalogInsGeneric)
