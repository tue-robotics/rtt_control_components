/** DoublesToJointState.hpp
 *
 * @class DoublesToJointState
 *
 * \author Janno Lunenburg
 * \date Aug, 2013
 * \version 1.0
 *
 */

#ifndef DOUBLETOJOINTSTATE_HPP
#define DOUBLETOJOINTSTATE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

#define maxN 40 //Maximum  size

using namespace std;
using namespace RTT;

namespace ROS
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class DoublesToJointState
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output ports*/
    OutputPort<sensor_msgs::JointState> outport_;

    InputPort<doubles> position_inport_;
    InputPort<doubles> velocity_inport_;
    InputPort<doubles> effort_inport_;

    /* Declaring global variables */
    uint Ndouble_; // Number of doubles in vector

    sensor_msgs::JointState out_msg_;

    public:

    DoublesToJointState(const string& name);
    ~DoublesToJointState();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
