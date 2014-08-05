#ifndef ANALOGOUTS_HPP
#define ANALOGOUTS_HPP

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

namespace SOEM // Just because it looks nice
{
  typedef std::vector<double> doubles;
  typedef std::vector< std::vector<double> > doubless;

  /*! \class Homing
  *  \brief Defines Orocos component for homing hardware component
  *
  *  This AnalogOuts component can be configured to map inputs
  *  on outputs.
  *
  *  To configure this component, the input_vector and the output_vector
  *  should be provided. If for example the input consists of two ports
  *  respectively of size 4 and 1. Than the input vector should be [4 1]
  *  Equally, if there are 3 outports of size 3 than the output_vector
  *  should be [3 3 3].
  *
  *  Note that at the moment it is not possible to do completely free
  *  mapping, the order of inputs is for the in and output the same. Also
  *  every output is subsequent to the previous so it is not possible to
  *  leave an output unused.
  *
  */

  class AnalogOutsGeneric
  : public RTT::TaskContext
    {
    private:

    // ports
    OutputPort<soem_beckhoff_drivers::AnalogMsg> outports[maxN];
    InputPort<doubles> inports[maxN];
    
    // Properties
    doubles input_vector;
    doubles output_vector;

    // Local variables:
    uint n_inports;
    uint n_outports;
    uint n_inputs;
    uint n_outputs;
    doubless inputdata;
    doubless outputdata;
    doubless mapping;
   
    // Global variables:
    std::vector<soem_beckhoff_drivers::AnalogMsg> analog_msgs;

    public:

    AnalogOutsGeneric(const string& name);
    ~AnalogOutsGeneric();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
