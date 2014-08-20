#ifndef ANALOGINSGENERIC_HPP
#define ANALOGINSGENERIC_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <std_msgs/Float32.h>

#define maxN 10 // maximum number of inputs and outputs

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace SOEM
{
  typedef std::vector<double> doubles;
  typedef std::vector< std::vector<double> > doubless;

/*! \class AnalogInsGeneric
*  \brief Defines a configurable Orocos component for Analog Inputs
*
*  To configure this component the following parameters should be 
*  provided:
* 
*  numberofinports, numberofoutports
*  input_sizes, output_sizes
*  input_positions
* 
*  If for example the input consists of three ports of size three
*  than the input_sizes should be [3 3 3], with numberofinports 3
*  Equally, if there is one outport of size 8
*  than output_sizes should be [8], with numberofoutports 1
* 
*  Suppose we want to map as shown underneath:
*  IN[size]:	OUT[size]:
*  input1[3]	output1[8]	(first two elements of input1 to first two elements of output1)
*  input2[3]    output1[8]	(three elements of input2 to element 3,4,5 of output1)
*  input3[3]	output1[8]  (three elements of input3 to element 6,7,8 of output1)
*  
*  Than our input_positions should look like:
*  output_positions = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
*  (size of input_positions should be equal to sum of input_sizes)
*  
*  Note that only sequential mapping is supported. You can not freely
*  map inputs to outputs. Only skip certain outputs.
*  
*  Inputs of this component are doubles
*  Outputs of this component are analog msgs
*
*  Inputs of this component are analog msgs
*  Outputs of this component are doubles
*
*  This component is similar to AnalogOuts except data types of input
*  and output ports are reversed. And this component also has the ability
*  to directly stream the output to ROS. (different output data type)
*/

  class AnalogInsGeneric
  : public RTT::TaskContext
    {
    private:

    // ports
    InputPort<soem_beckhoff_drivers::AnalogMsg> inports[maxN];
    OutputPort<doubles> outports[maxN];
    OutputPort<std_msgs::Float32> outports_toROS[maxN];

    // Properties
    uint n_inports;
    uint n_outports;
    doubles input_sizes;
    doubles output_sizes;
    doubles input_positions;
    bool direct_to_ROS;

    // Local variables:
    uint n_inputs;
    uint n_outputs;
    doubless mapping;

    // Global variables:
    std::vector<soem_beckhoff_drivers::AnalogMsg> inputdata_msgs;
    doubless outputdata;
    std_msgs::Float32 outputdata_msg;

    public:

    AnalogInsGeneric(const string& name);
    ~AnalogInsGeneric();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
