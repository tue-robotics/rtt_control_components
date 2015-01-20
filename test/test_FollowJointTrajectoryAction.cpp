// Author: Rob Janssen & the two stooges

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

//#include <tue_manipulation/GraspPrecomputeAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_grasp");
    Client client("/joint_trajectory_action_left", true); // true -> don't need ros::spin()
    client.waitForServer();
    control_msgs::FollowJointTrajectoryGoal goal;

    ros::Time::waitForValid(ros::WallDuration(2));

    goal.trajectory.header.stamp = ros::Time::now();

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
          as a single trajectory. Alternatively, each of these waypoints could
          be in its own trajectory - a trajectory can have one or more waypoints
          depending on the desired application.
          */

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("shoulder_pitch_joint");
        goal.trajectory.joint_names.push_back("shoulder_yaw_joint");
        goal.trajectory.joint_names.push_back("shoulder_roll_joint");
        goal.trajectory.joint_names.push_back("elbow_pitch_joint");
        goal.trajectory.joint_names.push_back("wrist_roll_joint");
        goal.trajectory.joint_names.push_back("wrist_pitch_joint");
        goal.trajectory.joint_names.push_back("wrist_yaw_joint");

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(2);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = 0.0;
        goal.trajectory.points[ind].positions[1] = 0.0;
        goal.trajectory.points[ind].positions[2] = 0.0;
        goal.trajectory.points[ind].positions[3] = 0.0;
        goal.trajectory.points[ind].positions[4] = 0.0;
        goal.trajectory.points[ind].positions[5] = 0.0;
        goal.trajectory.points[ind].positions[6] = 0.0;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Second trajectory point
        // Positions
        ind += 1;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = -0.3;
        goal.trajectory.points[ind].positions[1] = 0.2;
        goal.trajectory.points[ind].positions[2] = -0.1;
        goal.trajectory.points[ind].positions[3] = -1.2;
        goal.trajectory.points[ind].positions[4] = 1.5;
        goal.trajectory.points[ind].positions[5] = -0.3;
        goal.trajectory.points[ind].positions[6] = 0.5;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 2 seconds after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(20.0);


        client.sendGoal(goal);
        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Grasp precompute successful\n");
        else
            printf("Grasp precompute unsuccesfull\n");

        return 0;
    }
