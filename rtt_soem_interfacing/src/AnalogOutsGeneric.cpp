#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogOutsGeneric.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogOutsGeneric::AnalogOutsGeneric(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "input_vector", input_vector ).doc("Vector specifying input sizes. For amigo's base + spindle [4, 1] and for peras [9]");
    addProperty( "output_vector", output_vector ).doc("Vector specifying output sizes. For amigo's base + spindle [8] and for peras [3, 3, 3]");
}
AnalogOutsGeneric::~AnalogOutsGeneric(){}

bool AnalogOutsGeneric::configureHook()
{
    if (input_vector.size()  == 0 || output_vector.size()  == 0 ) {
        log(Error) << "AnalogOutsGeneric: Either the input_vector or the output_vector is not correctly specified. " << endlog();
        log(Error) << "AnalogOutsGeneric: Please make sure these properties are set before the call to the configureHook()" << endlog();
        return false;
    }

    // Determine number of ports
    n_inports = input_vector.size();
    n_outports = output_vector.size();

    // Determine number of inputs
    n_inputs = 0;
    n_outputs = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        n_inputs += input_vector[i];
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        n_outputs += output_vector[i];
    }

    // resizing of variables
    inputdata.resize(n_inports);
    for ( uint i = 0; i < n_inports; i++ ) {
        inputdata[i].resize(input_vector[i]);
    }

    outputdata.resize(n_outports);
    analog_msgs.resize(n_outports);
    for ( uint i = 0; i < n_outports; i++ ) {
        outputdata[i].resize(output_vector[i]);
        analog_msgs[i].values.resize(output_vector[i]);
    }

    // Create ports
    for ( uint i = 0; i < n_inports; i++ ) {
        string name_inport = "in"+to_string(i+1);
        addEventPort( name_inport, inports[i] );
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        string name_outport = "out"+to_string(i+1);
        addPort( name_outport, outports[i] );
    }

    // Create mappingsmatrix
    mapping.resize(n_inputs);
    for ( uint i = 0; i < n_inputs; i++ ) {
        mapping[i].assign(4,0.0);
    }

    uint k = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        for ( uint j = 0; j < input_vector[i]; j++ ) {
            mapping[k][0] = i;
            mapping[k][1] = j;
            k++;
        }
    }

    k = 0;
    for ( uint i = 0; i < n_outports; i++ ) {
        for ( uint j = 0; j < output_vector[i]; j++ ) {
            mapping[k][2] = i;
            mapping[k][3] = j;
            k++;
        }
    }

    return true;
}

bool AnalogOutsGeneric::startHook()
{
	return true;
}

void AnalogOutsGeneric::updateHook()
{
    // check all ports for newdata
    for ( uint i = 0; i < n_inports; i++ ) {
        if ( NewData == inports[i].read(inputdata[i])) { }
    }

    // convert inputdata to analog_msgs
    uint k = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        for ( uint j = 0; j < input_vector[i]; j++ ) {
            uint map_in_1 = mapping[k][0];
            uint map_in_2 = mapping[k][1];
            uint map_out_1 = mapping[k][2];
            uint map_out_2 = mapping[k][3];
            analog_msgs[map_out_1].values[map_out_2] = inputdata[map_in_1][map_in_2];
            k++;
        }
    }

    // sending msgs to outports // to do test if outports.write(analog_msgs); would also work
    for ( uint i = 0; i < n_outports; i++ ) {
        //log(Warning) << "AnalogOutsGeneric: i: [" << i << "]" << endlog();
        outports[i].write(analog_msgs[i]);
    }
}

ORO_CREATE_COMPONENT(SOEM::AnalogOutsGeneric)
