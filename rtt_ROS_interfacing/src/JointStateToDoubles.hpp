/** JointStateToDoubles.hpp
 *
 * @class DoublesToJointState
 *
 * \author Janno Lunenburg
 * \date Aug, 2013
 * \version 1.0
 *
 */

#ifndef JOINTSTATETODOUBLES_HPP
#define JOINTSTATETODOUBLES_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace ROS
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class JointStateToDoubles
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output ports*/
    InputPort<sensor_msgs::JointState> inport_;
	InputPort<doubles> position_inport_;

    OutputPort<doubles> position_outport_;
    OutputPort<doubles> velocity_outport_;
    OutputPort<doubles> effort_outport_;

    /* Declaring global variables */
    uint Ndouble_; // Number of doubles in vector
    uint Ndouble2_;
    doubles pos_out_, vel_out_, eff_out_, pos_in_;

    public:

    JointStateToDoubles(const string& name);
    ~JointStateToDoubles();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
