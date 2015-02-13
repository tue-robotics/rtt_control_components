#ifndef ANALOGINSGENERIC_HPP
#define ANALOGINSGENERIC_HPP

/* C++ includes */
#include <vector>
#include <string>

/* OROCOS includes */
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

/* OROCOS SOEM includes */
#include <soem_beckhoff_drivers/AnalogMsg.h>

/* ROS includes */
#include <std_msgs/Float32MultiArray.h>


/*
                                                              _______________________
  soem_beckhoff_drivers::AnalogMsg in_1 = { x, y, z }        |                       | -----> std::vector<double> std_out_1 = {z, b, x}
                        -----------------------------------> |   AnalogInsGeneric    | -----> std_msgs::Float32Multiarray ros_out_1 = {z, b, x}
  soem_beckhoff_drivers::AnalogMsg in_2 = { a, b, c, d, e }  |    (example config)   | -----> std::vector<double> std_out_2 = {a, e}
                        -----------------------------------> |_______________________| -----> std_msgs::Float32Multiarray ros_out_2 = {a, e}
  Robin, 9 Feb 2015
  This component maps input messages of type soem_beckhoff_drivers::AnalogMsg to a configurable set of
  outports. Each outport is available twice: Once as an outport sending std::vector<double> messages and
  once as an outport sending ROS std_msgs::Float32MultiArray messages. The dimensions of in- and outports
  and the mapping of specific inport entries to specific outport entries can be configured via four properties:
    .inport_dimensions
        Array specifying, for each inport, the number of entries a single soem_beckhoff_driver::AnalogInMsg
        contains. For instance: {2,3,3} creates three inports, the first one expects AnalogInMsg-messages of
        length 2, the second and third expect AnalogInMsg-messages of length 3.
    .outport_dimensions
        Array specifying, for each outport, the number of entries a single message contains. For instance:
        {2,8,8} creates three std::vector and three ROS msg outports. For both outport types the first port
        would send messages of length 2, and the second and third outport would send messages of length 8.
    .from_which_inport
        Array specifying, for each entry in the outport messages, from which inport its value should come. For
        instance: If inport_dimensions is set to {2,1}, and outport_dimensions to {1,2,1}, then setting
        from_which_inport to {1,2,2,1} implies that the first and third outport messages contain data coming
        from the first inport, while both entries in messages from the second outport are coming from the second
        inport.
    .from_which_entry
        Array specifying, for each entry in the outport messages, which entry from the inport specified in
        from_which_inport it should contain. For instance: If inport_dimensions is set to {2,1},
        outport_dimensions to {1,2,1} and from_which_inport to {1,2,2,1}, then setting from_which_entry to
        {2,1,1,1} implies that the first outport contains the second entry of the first inport, both entries
        of the second outport contain the first entry of the second inport and the third outport contains the
        first entry of the first inport
*/

#define MAX_PORTS 10 /* maximum number of ports (inports + outports) */

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace SOEM
{

class AnalogInsGeneric
        : public RTT::TaskContext
{
private:

    uint n_inports, n_outports, n_inport_entries, n_outport_entries;

    // Orocos ports
    InputPort<soem_beckhoff_drivers::AnalogMsg> inports[MAX_PORTS];
    OutputPort< std::vector<double> > stdvect_outports[MAX_PORTS];
    OutputPort<std_msgs::Float32MultiArray> rosmsg_outports[MAX_PORTS];

    // Orocos properties
    std::vector<double> inport_dimensions, outport_dimensions;
    std::vector<double> from_which_inport, from_which_entry;

public:

    AnalogInsGeneric(const string& name);

    bool configureHook();
    bool startHook();
    void updateHook();

    ~AnalogInsGeneric();
};
}
#endif
