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
*  To configure this component, the input_sizes and the output_sizes
*  should be provided. If for example the input consists of two ports
*  respectively of size 4 and 1. Than the input vector should be [4 1]
*  Equally, if there are 3 outports of size 3 than the output_sizes
*  should be [3 3 3].
*
*  Note that at the moment it is not possible to do completely free
*  mapping, the order of inputs is for the in and output the same. Also
*  every output is subsequent to the previous so it is not possible to
*  skip an unused input or output.
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
