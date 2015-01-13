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
#include <actionlib/action_definition.h>
#include <rtt_actionlib_examples/SomeActionAction.h>




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
