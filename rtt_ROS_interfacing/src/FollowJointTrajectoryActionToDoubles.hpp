/** FollowJointTrajectoryActionToDoubles.hpp
 *
 * @class FollowJointTrajectoryActionToDoubles
 *
 * \author Tim Clephas
 * \date May, 2014
 * \version 1.0
 *
 */

#ifndef FollowJointTrajectoryActionToDoubles_HPP
#define FollowJointTrajectoryActionToDoubles_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

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

  class FollowJointTrajectoryActionToDoubles
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;

    /* Declaring and output ports*/
    InputPort<control_msgs::FollowJointTrajectoryActionGoal> goalport;
    OutputPort<control_msgs::FollowJointTrajectoryActionResult> resultport;

    OutputPort<doubles> position_outport_;
    OutputPort<doubles> effort_outport_;

    /* Declaring global variables */
    uint Ndouble_; // Number of doubles in vector
    double max_dx;
    doubles last_pos_out_, pos_out_, eff_out_, cur_max_acc, cur_max_vel;
    doubles joint_states;
    	doubles goal_pos, pos;
    uint tp;
    control_msgs::FollowJointTrajectoryActionGoal goalmsg;
    //trajectory_msgs::JointTrajectory goal;
    bool playing_trajectory;
    bool playing_trajectory_point;
    
	double abs(double v, double& s) {
		if (v >= 0) {
			s = 1.0;
			return v;
		} else {
			s = -1.0;
			return -v;
		}
	}

    public:

    FollowJointTrajectoryActionToDoubles(const string& name);
    ~FollowJointTrajectoryActionToDoubles();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
