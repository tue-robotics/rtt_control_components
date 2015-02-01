#ifndef TrajectoryActionlib_HPP
#define TrajectoryActionlib_HPP

#define maxN 10 //Maximum matrix size

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

#include <amigo_ref_interpolator/interpolator.h>
#include <sensor_msgs/JointState.h>

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace ROS
{

    class TrajectoryActionlib
		: public RTT::TaskContext
		{
		private:
    
			ACTION_DEFINITION(control_msgs::FollowJointTrajectoryAction)
			// Convenience typedefs
            typedef vector<double> doubles;
            typedef vector<int> ints;


			// Declaring input- and output_ports
			InputPort<doubles> posinport[5];
			InputPort<doubles> initialposinport;
			OutputPort<doubles> posoutport;
			OutputPort<doubles> veloutport;
			OutputPort<doubles> accoutport;
			OutputPort<sensor_msgs::JointState> resetrefoutport;

			// Properties
			uint N;
			uint N_inports;
			doubles minpos;
			doubles maxpos;
			doubles maxvel;
			doubles maxacc;
			ints inport_sizes;

			// Declaring global variables
			std::vector<refgen::RefGenerator> mRefGenerators;
			std::vector<amigo_msgs::ref_point> mRefPoints;
			doubles desiredPos;
			doubles desiredVel;
			doubles desiredAcc;
			double InterpolDt, InterpolEps;

			// RTT action server
			rtt_actionlib::RTTActionServer<control_msgs::FollowJointTrajectoryAction> rtt_action_server_;
			typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
			GoalHandle current_gh_;
			Feedback feedback_;
			Result result_;

		public:

            TrajectoryActionlib(const string& name);
            ~TrajectoryActionlib();

			bool configureHook();
			bool startHook();
			void updateHook();
            void resetReference();
            void goalCallback(GoalHandle gh);
            void cancelCallback(GoalHandle gh);
	};
}
#endif
