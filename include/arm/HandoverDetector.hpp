#ifndef HANDOVERDETECTOR_HPP
#define HANDOVERDETECTOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tue_msgs/GripperCommand.h>
#include <rtt_actionlib/rtt_actionlib.h>
#include <rtt_actionlib/rtt_action_server.h>
#include <actionlib/action_definition.h>

using namespace std;
using namespace RTT;

namespace ARM
{
  typedef vector<double> doubles;
  
  class HandoverDetector
  : public RTT::TaskContext
    {
    private:

	// Inputports
	InputPort<std_msgs::Bool> toggle_h2r_inport;
	InputPort<std_msgs::Bool> toggle_r2h_inport;
	InputPort<doubles> torque_inport;
	
	// Outputports
	OutputPort<std_msgs::Bool> result_outport;
	OutputPort<tue_msgs::GripperCommand> gripper_outport;
	
	// Properties
	double threshold;

	// Msgs
	std_msgs::Bool toggle_msg;
	tue_msgs::GripperCommand gripperCommand_msg;

	// Variables
	bool toggled_r2h;
	bool toggled_h2r;	
	double lowerthreshold;
	double upperthreshold;
	doubles torques;
	double sign;
	
    public:

    HandoverDetector(const string& name);
    ~HandoverDetector();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
