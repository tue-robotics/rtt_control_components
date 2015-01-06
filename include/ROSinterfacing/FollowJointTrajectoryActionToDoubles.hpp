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
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <rtt_actionlib_examples/SomeActionAction.h>


#define maxN 40 //Maximum  size

template <typename T> int sgn(T val) {
return (T(0) < val) - (val < T(0));
}

using namespace std;
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
    ACTION_DEFINITION(control_msgs::FollowJointTrajectoryAction);

    // Convenience typedefs
    typedef vector<double> doubles;
    typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
    GoalHandle current_gh_;
    Feedback feedback_;
    Result result_;

    // RTT action server
    rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> rtt_action_server_;


    /* Declaring and output ports*/
    InputPort<control_msgs::FollowJointTrajectoryActionGoal> goalport;
    OutputPort<control_msgs::FollowJointTrajectoryActionResult> resultport;

    OutputPort<doubles> position_outport_;
    InputPort<doubles> resetPort;

    /* Declaring global variables */
    uint Nj; // Number of doubles in vector
    double max_dx;
    doubles cur_max_acc, cur_max_vel;
    doubles goal_pos;
    doubles start_pos;
    doubles pos, vel;

    doubles max_vels, max_accs;
    uint tp;
    uint slowest;
    control_msgs::FollowJointTrajectoryActionGoal goalmsg;
    bool playing_trajectory;
    bool playing_trajectory_point;

    public:

    FollowJointTrajectoryActionToDoubles(const string& name);
    ~FollowJointTrajectoryActionToDoubles();

    bool configureHook();
    bool startHook();
    void updateHook();
    void goalCallback(GoalHandle gh);
    void cancelCallback(GoalHandle gh);
    };
}
#endif
