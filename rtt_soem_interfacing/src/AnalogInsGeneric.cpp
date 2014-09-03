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
    addProperty( "input_positions", input_positions ).doc("Vector specifying which inputs are obsolete");
    addProperty( "direct_to_ROS", direct_to_ROS ).doc("Boolean specifying wether the output has to be streamed directly to ROS");
}
AnalogInsGeneric::~AnalogInsGeneric(){}

bool AnalogInsGeneric::configureHook()
{
	// Determine number of in- and outputs
    n_inputs = 0;
    n_outputs = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        n_inputs += input_sizes[i];
    }
    for ( uint i = 0; i < n_outports; i++ ) {
        n_outputs += output_sizes[i];
    }
    uint nonobsolete_input_positions = 0;
    for ( uint i = 0; i < input_positions.size(); i++ ) {
        if (input_positions[i] != 0.0) {
            nonobsolete_input_positions++;
        }
    }

    // Verify property feasibility
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
    if ( input_positions.size() != n_inputs) {
        log(Error) << "AnalogInsGeneric: The size of output_positions does not match n_outputs (which is the sum of all elements of output_sizes)" << endlog();
        return false;
    }
    if ( nonobsolete_input_positions != n_outputs) {
        log(Error) << "AnalogInsGeneric:  " << nonobsolete_input_positions << " non zero entries in input_positions is not equal to n_outputs: " << n_outputs << "!" << endlog();
        return false;
    }
    if (direct_to_ROS) {
        for ( uint i = 0; i < n_outports; i++ ) {
            if (output_sizes[i] != 1.0) {
                log(Error) << "AnalogInsGeneric: direct_to_ROS is true, But one of the output sizes is not equal to 1.0" << endlog();
                return false;
            }
        }
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
        string name_outport_toROS = "outmsg"+to_string(i+1);
        addPort( name_outport_toROS, outports_toROS[i] );
    }

    // Create empty mapping matrix - each row maps one input to one output
    mapping.resize(max(n_inputs,n_outputs));
    for ( uint i = 0; i < max(n_inputs,n_outputs); i++ ) {
        mapping[i].assign(4,0.0);
    }
    // Fill first two columns of mapping matrix - First column specifies which inport and the second column specifies which double from that input has to be mapped
    uint k = 0;
    uint l = 0;
    for ( uint i = 0; i < n_inports; i++ ) {
        for ( uint j = 0; j < input_sizes[i]; j++ ) {
            if (k - l < min(n_inputs,n_outputs)) {
                if (input_positions[k] != 0.0) {
                    mapping[k-l][0] = i;
                    mapping[k-l][1] = j;
                } else {
                    l++;
                }
            }
            k++;
        }
    }

    // Fill last two columns of mapping matrix -  Third column specifies which output and the fourth column specifies to which double in the vector of that output
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
    
    //for ( uint m = 0; m < max(n_inputs,n_outputs); m++ ) {
		//log(Warning) << "AI: mapping matrix is: [" << mapping[m][0] << "," << mapping[m][1] << "," << mapping[m][2] << "," << mapping[m][3] << "]" << endlog();		
	//}

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
    if (direct_to_ROS) {
        for ( uint i = 0; i < n_outports; i++ ) {
            if ( !outports_toROS[i].connected() ) {
              log(Warning)<<"AnalogInsGeneric:: outmsg"<< i+1 <<" not connected!"<<endlog();
            }
        }
    } else {
        for ( uint i = 0; i < n_outports; i++ ) {
            if ( !outports[i].connected() ) {
              log(Warning)<<"AnalogInsGeneric:: out"<< i+1 <<" not connected!"<<endlog();
            }
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

ORO_CREATE_COMPONENT(SOEM::AnalogInsGeneric)
