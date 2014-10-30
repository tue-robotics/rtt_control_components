#ifndef ANALOGOUTSGENERIC_HPP
#define ANALOGOUTSGENERIC_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>

#define maxN 5 // maximum number of inputs and outputs, 

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

/*! \class AnalogOutsGeneric
*  \brief Defines a configurable Orocos component for Analog Outputs
*
*  To configure this component the following parameters should be 
*  provided:
* 
*  numberofinports, numberofoutports
*  input_sizes, output_sizes
*  output_positions
* 
*  If for example the input consists of two ports respectively of size 
*  4 and 1 than the input_sizes should be [4 1], with numberofinports 2
*  Equally, if there are three outports of size three than output_sizes  
*  should be [3 3 3], with numberofoutports 3
* 
*  Suppose we want to map as shown underneath:
*  IN[size]:	OUT[size]:
*  input1[4]	ouput1[3]	(First three elements of input1)
* 				output2[3]	(Last element of input1 + two obs outputs)
*  input2[1]	output3[3]	(First Element of input2 + two obs outputs)
*  
*  Than our output_positions should look like:
*  output_positions = [1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
*  (size of output_positions should be equal to sum of output_sizes)
*  
*  Note that only sequential mapping is supported. You can not freely
*  map inputs to outputs. Only skip certain outputs.
*  
*  Inputs of this component are doubles
*  Outputs of this component are analog msgs
*
*  This component is similar to AnalogIns except data types of input
*  and output ports are reversed.
*/

  class AnalogOutsGeneric
  : public RTT::TaskContext
    {
    private:

    // ports
    InputPort<doubles> inports[maxN];
    OutputPort<soem_beckhoff_drivers::AnalogMsg> outports[maxN];

    // Properties
    uint n_inports;
    uint n_outports;
    doubles input_sizes;
    doubles output_sizes;
    doubles output_positions;

    // Local variables:
    uint n_inputs;
    uint n_outputs;
    doubless mapping;

    // Global variables:
    doubless inputdata;
    std::vector<soem_beckhoff_drivers::AnalogMsg> outputdata_msgs;

    public:

    AnalogOutsGeneric(const string& name);
    ~AnalogOutsGeneric();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
