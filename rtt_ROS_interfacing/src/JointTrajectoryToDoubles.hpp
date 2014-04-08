/** JointTrajectoryToDoubles.hpp
 *
 * @class DoublesToJointState
 *
 * \author Janno Lunenburg
 * \date Aug, 2013
 * \version 1.0
 *
 */

#ifndef JOINTTRAJECTORYTODOUBLES_HPP
#define JOINTTRAJECTORYTODOUBLES_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#define maxN 40 //Maximum  size. Still a workaround.

using namespace std;

/*template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};*/

using namespace RTT;

namespace ROS
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class JointTrajectoryToDoubles
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    //typedef vector<string> strings;

    /* Declaring and output ports*/
    InputPort<trajectory_msgs::JointTrajectory> inport_;

    OutputPort<doubles> position_outport_;
    OutputPort<doubles> effort_outport_;

    /* Declaring global variables */
    uint Ndouble_; // Number of doubles in vector
    double max_dx;
    doubles last_pos_out_, pos_out_, eff_out_; 
    uint tp;
    trajectory_msgs::JointTrajectory in_msg;
    bool playing_trajectory;


    public:

    JointTrajectoryToDoubles(const string& name);
    ~JointTrajectoryToDoubles();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
