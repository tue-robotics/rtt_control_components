/** BenchmarkReference.hpp
 *
 * @class BenchmarkReference
 *
 * \author Max Baeten
 * \date April, 2014
 * \version 1.0
 *
 */

#ifndef BENCHMARKREFERENCE_HPP
#define BENCHMARKREFERENCE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

#define N 7

using namespace std;
using namespace RTT;

namespace ROS
{
  class BenchmarkReference
  : public RTT::TaskContext
    {
    private:
    
    int cntr;

    typedef vector<double> doubles;

    /* Declaring output ports*/
    OutputPort<doubles> position_outport_;
    OutputPort<doubles> velocity_outport_;
    OutputPort<doubles> effort_outport_;

    /* Declaring global variables */
    doubles REF0_, REF1_, REF2_, REF3_, REF4_, REF5_, REF6_, REF7_, REF8_, REF9_, REF10_, pos_out_;

    public:

    BenchmarkReference(const string& name);
    ~BenchmarkReference();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
