#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "AnalogInsGeneric.hpp"

using namespace std;
using namespace RTT;
using namespace SOEM;

AnalogInsGeneric::AnalogInsGeneric(const string& name) : TaskContext(name, PreOperational)
{
    addProperty( "inport_dimensions", inport_dimensions ).doc("Array specifying, for each inport, the number of entries a single "
                                                              "soem_beckhoff_driver::AnalogInMsg contains.");

    addProperty( "outport_dimensions", outport_dimensions ).doc("Array specifying, for each outport, the number of entries a single "
                                                                "std::vector<double> or ROS std_msgs::Float32Multiarray message contains.");

    addProperty( "from_which_inport", from_which_inport ).doc("Array specifying, for each entry in the outport messages, from which inport "
                                                              "its value should come.");

    addProperty( "from_which_entry", from_which_entry ).doc("Array specifying, for each entry in the outport messages, which entry from the "
                                                            "inport specified in from_which_inport it should contain.");
}

bool AnalogInsGeneric::configureHook()
{

    /***************************************
        Check for illegal configuration
    /***************************************/

    bool error = false;
    n_inports = inport_dimensions.size();

    if(n_inports < 1)
    {
        log(Error) << "In AnalogInsGeneric: Need at least one inport to configure this component!" << endlog();
        error = true;
    }

    n_inport_entries = 0;

    for(uint i = 0; i < n_inports; ++i)
    {
        if(inport_dimensions[i] < 1)
        {
            log(Error) << "In AnalogInsGeneric: Inport_dimensions cannot contain value smaller than one!" << endlog();
            error = true;
        }
        n_inport_entries += inport_dimensions[i];
    }

    n_outports = outport_dimensions.size();

    if(n_outports < 1)
    {
        log(Error) << "In AnalogInsGeneric: Need at least one outport to configure this component!" << endlog();
        error = true;
    }

    n_outport_entries = 0;

    for(uint i = 0; i < n_outports; ++i)
    {
        if(outport_dimensions[i] < 1)
        {
            log(Error) << "In AnalogInsGeneric: Outport_dimensions cannot contain value smaller than one!" << endlog();
            error = true;
        }

        n_outport_entries += outport_dimensions[i];
    }
    if(n_outports + n_inports > MAX_PORTS)
    {
        log(Error) << "In AnalogInsGeneric: Too many in- and outports specified! MAX_PORTS = " << MAX_PORTS << endlog();
        error = true;
    }

    if(from_which_inport.size() != n_outport_entries)
    {
        log(Error) << "In AnalogInsGeneric: The number of entries in from_which_inport should equal the total number of output values." << endlog();
        error = true;
    }

    if(from_which_entry.size() != n_outport_entries)
    {
        log(Error) << "In AnalogInsGeneric: The number of entries in from_which_entry should equal the total number of output values." << endlog();
        error = true;
    }

    for(uint i = 0; i < n_outport_entries; ++i)
    {
        if( from_which_inport[i] > n_inports || from_which_inport[i] <= 0 )
        {
            log(Error) << "In AnalogInsGeneric: From_which_inport array contains port no. " << from_which_inport[i] << " which does not exist according to inport_dimensions!" << endlog();
            error = true;
        }
        else if ( from_which_entry[i] > inport_dimensions[ from_which_inport[i]-1 ] || from_which_entry[i] <= 0 )
        {
            log(Error) << "In AnalogInsGeneric: From_which_entry array contains entry no. " << from_which_entry[i] << " which does not exist for inport no. " << from_which_inport[i] << "!" << endlog();
            error = true;
        }
    }

    if( error )
        return false;


    /***************************************
        Create in- and outports
    /***************************************/


    for( uint i = 0; i < n_inports; i++ )
        addEventPort( "beckhoffmsg_in_"+to_string(i+1), inports[i] );


    for( uint i = 0; i < n_outports; i++ )
    {
        addPort( "stdvect_out_"+to_string(i+1), stdvect_outports[i] );
        addPort( "rosmsg_out_"+to_string(i+1), rosmsg_outports[i] );
    }

    return true;
}

bool AnalogInsGeneric::startHook()
{
    for(uint i = 0; i < n_inports; ++i)
    {
        if ( !inports[i].connected() )
        {
            log(Error) << "In AnalogInsGeneric: Inport " << inports[i].getName() << " is not connected, cannot start component" << endlog();
            return false;
        }
    }
    return true;
}

void AnalogInsGeneric::updateHook()
{

    /***************************************
        Initialize in- and outport messages
    /***************************************/

    std::vector< soem_beckhoff_drivers::AnalogMsg > inputdata_msgs;
    std::vector< std::vector< double > >outputdata_std_vect;
    std::vector< std_msgs::Float32MultiArray > outputdata_ros_msg;

    inputdata_msgs.resize(n_inports);

    for( uint i = 0; i < n_inports; ++i )
        inputdata_msgs[i].values.resize( inport_dimensions[i] );

    outputdata_std_vect.resize(n_outports);
    outputdata_ros_msg.resize(n_outports);

    for( uint i = 0; i < n_outports; i++ )
    {
        outputdata_std_vect[i].resize( outport_dimensions[i] );
        outputdata_ros_msg[i].data.resize( outport_dimensions[i] );
    }


    /******************************************
        Loop over inports to obtain new data
    /******************************************/

    for( uint i = 0; i < n_inports; ++i )
    {
        switch( inports[i].read(inputdata_msgs[i]) )
        {
        case NewData:
            break;
        case OldData:
            log(Info) << "In AnalogInsGeneric: Using old data for inport " << inports[i].getName() << endlog();
            break;
        case NoData:
            log(Error) << "In AnalogInsGeneric: Could not read data for inport " << inports[i].getName() << endlog();
            return;
        default:
            log(Error) << "In AnalogInsGeneric: Inport.read() returns unknown status!" << endlog();
            return;
        }
    }

    /***************************************
        Loop over outports, do mapping
    /***************************************/

    uint k = 0;
    for( uint i = 0; i < n_outports; ++i )
    {
        for( uint j = 0; j < outport_dimensions[i]; ++j)
        {
            outputdata_std_vect[i][j] = inputdata_msgs[ from_which_inport[k]-1 ].values[ from_which_entry[k]-1 ];
            outputdata_ros_msg[i].data[j] = inputdata_msgs[ from_which_inport[k]-1 ].values[ from_which_entry[k]-1 ];
            ++k;
        }
        rosmsg_outports[i].write(outputdata_ros_msg[i]);
        stdvect_outports[i].write(outputdata_std_vect[i]);
    }

}

AnalogInsGeneric::~AnalogInsGeneric()
{
    // destructor
}

ORO_CREATE_COMPONENT(SOEM::AnalogInsGeneric)
