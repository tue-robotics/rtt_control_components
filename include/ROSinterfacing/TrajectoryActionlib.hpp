#ifndef TrajectoryActionlib_HPP
#define TrajectoryActionlib_HPP

#define maxN 5 //Maximum number of bodyparts

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
#include <tue/manipulation/reference_generator.h>


#include <amigo_ref_interpolator/interpolator.h>

#include <queue>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
}

typedef trajectory_msgs::JointTrajectoryPoint Point;

using namespace RTT;

namespace ROS
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
    typedef vector<bool> bools;
    typedef vector<string> strings;
    typedef pair<int, int> BodyJointPair;

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

			// Convenience typedefs
			typedef vector<double> doubles;
			typedef vector<int> ints;
			typedef vector<bool> bools;
			typedef vector<string> strings;
			typedef pair<int, int> BodyJointPair;

			// Declaring input- and output_ports
            InputPort<doubles> currentpos_inport[maxN];
			OutputPort<doubles> posoutport[maxN];
			OutputPort<doubles> veloutport[maxN];
            OutputPort<doubles> accoutport[maxN];

			// Properties
            vector<doubles> minpos;
            vector<doubles> maxpos;
            vector<doubles> maxvel;
            vector<doubles> maxacc;

            // Global variables - scalar
            bool checked;
            int totalNumberOfJoints;
            int numberOfBodyparts;
            double start_time;
            bool trajectory_active;

            // Global variables - vector
            bools allowedBodyparts;
            bools allowedBodyparts_prev;
            ints activeBodyparts;
            ints vector_sizes;
            doubles InterpolDts;
            doubles InterpolEpses;

            // Global variables - vector of vectors
            vector<doubles> actualPos;
            vector<doubles> desiredPos;
            vector<doubles> desiredVel;
            vector<doubles> desiredAcc;
            vector<doubles> pos_out;
            vector<doubles> vel_out;
            vector<doubles> acc_out;

            // Global variables - map
            map<string, BodyJointPair> joint_map;

            tue::manipulation::ReferenceGenerator reference_generator_;
            GoalHandle goal_handle_;
            bool has_goal_;
            double dt;

		public:

            TrajectoryActionlib(const string& name);
            ~TrajectoryActionlib();

			bool configureHook();
			bool startHook();
			void updateHook();
            void AddBodyPart(int partNr, strings JointNames);
            void SendToPos(int partNr, doubles pos);
            void ResetReferences(int partNr);
            bool CheckConnectionsAndProperties();
            void goalCallback(GoalHandle gh);
            void cancelCallback(GoalHandle gh);

	};
}
#endif
