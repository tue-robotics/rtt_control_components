#ifndef TrajectoryActionlib_HPP
#define TrajectoryActionlib_HPP

#define maxN 5 //Maximum bodyparts size

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/action_definition.h>
#include <rtt/Logger.hpp>
#include <urdf/model.h>


#include <amigo_ref_interpolator/interpolator.h>

#include <queue>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
}


using namespace RTT;

namespace ROS
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
    typedef vector<bool> bools;
    typedef vector<string> strings;
    typedef pair<int, int> BodyJointPair;

    /**
     * @brief Global component that receives trajectories for
     * all bodyparts. This component sorts them, and generates a
     * reference signal for all joints.
     *
     * The component has one input ports which should be connected with
     * stream to the joint state reference topic of the robot. For each
     * added bodypart an initial position input port is added and three
     * outputports for position, velocity and acceleration. If homable,
     * then also a second input port for homing references is added.
     *
     * Start this component in the robot's main ops file and add bodyparts
     * for each bodypart.
     *
     * Every bodypart can be in different states:
     * 1) Operational - The references from the Jointstate topic from
     *                  ROS are used.
     * 2) Homing      - The references from the homing port are used.
     * 3) Disabled    - The actual measured positions are sent as reference
     *                  since then the error is zero in the disabled mode
     *
     * The supervisor component enables and disables each bodypart.
     * The homing component switches a bodypart to the homing state
     *
     * Example for ops file:
     * TrajectoryActionlib.AddBodyPart(2, strings("spindle_joint") )
     * TrajectoryActionlib.minPos4 			= array ( 0.075)
     * TrajectoryActionlib.maxPos4 			= array ( 0.4)
     * TrajectoryActionlib.maxVel4 			= array ( 0.07)
     * TrajectoryActionlib.maxAcc4             = array ( 0.2)
     * TrajectoryActionlib.interpolatorDt4 	= 0.001
     * TrajectoryActionlib.interpolatorEps4 	= 1.0
     *
     */

    class TrajectoryActionlib
		: public RTT::TaskContext
		{
		private:
            // Actionlib stuff
			ACTION_DEFINITION(control_msgs::FollowJointTrajectoryAction)
            rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> rtt_action_server_;
            typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;

            Feedback feedback_;
            Result result_;

            struct TrajectoryInfo
            {
                TrajectoryInfo(const GoalHandle& gh) : goal_handle(gh), t_start(-1)
                {
                    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = gh.getGoal()->trajectory.points.begin(); it != gh.getGoal()->trajectory.points.end(); ++it)
                        points.push(*it);
                }

                double t_start;
                GoalHandle goal_handle;
                std::queue<trajectory_msgs::JointTrajectoryPoint> points;
            };

            std::vector < TrajectoryInfo > goal_handles_;

			// Convenience typedefs
			typedef vector<double> doubles;
			typedef vector<int> ints;
			typedef vector<bool> bools;
			typedef vector<string> strings;
			typedef pair<int, int> BodyJointPair;

            // iterators
            // j iterates over all connected bodyparts
            // i iterates over all joints within particular bodypart

			// Declaring input- and output_ports
            InputPort<doubles> currentpos_inport[maxN];
			OutputPort<doubles> posoutport[maxN];
			OutputPort<doubles> veloutport[maxN];
			OutputPort<doubles> accoutport[maxN];

			// Properties
            vector<doubles> minpos;
            vector<doubles> minpos2;
            vector<doubles> maxpos;
            vector<doubles> maxpos2;
            vector<doubles> maxvel;
            vector<doubles> maxvel2; //TODO: Remove maxvel and rename maxvel2 to maxvel
            vector<doubles> maxacc;
            vector<doubles> maxacc2;

            // Global variables - scalar
            bool checked;
            int totalNumberOfJoints;
            int numberOfBodyparts;
            double start_time;

            // Global variables - vector
            bools allowedBodyparts;
            bools allowedBodyparts_prev;
            ints activeBodyparts;
            ints vector_sizes;
            doubles InterpolDts;
            doubles InterpolEpses;

            // Global variables - vector of vectors
            vector<doubles> desiredPos;
            vector<doubles> desiredVel;
            vector<doubles> desiredAcc;
            vector<doubles> pos_out;
            vector<doubles> vel_out;
            vector<doubles> acc_out;
            vector<doubles> current_position;
            vector<vector<refgen::RefGenerator > > mRefGenerators;
            vector<vector<amigo_msgs::ref_point> > mRefPoints;

            // Global variables - map
            map<string, BodyJointPair> joint_map;

		public:

            TrajectoryActionlib(const string& name);
            ~TrajectoryActionlib();

			bool configureHook();
			bool startHook();
			void updateHook();
            void AddBodyPart(int partNr, strings JointNames);
            void SendToPos(int partNr, doubles pos);
            void ResetReference(int partNr);
            bool CheckConnectionsAndProperties();
            void goalCallback(GoalHandle gh);
            void cancelCallback(GoalHandle gh);
	};
}
#endif
