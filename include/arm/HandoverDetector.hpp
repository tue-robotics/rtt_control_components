#ifndef HANDOVERDETECTOR_HPP
#define HANDOVERDETECTOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace ARM
{
  typedef vector<double> doubles;
  
  class HandoverDetector
  : public RTT::TaskContext
    {
    private:

	InputPort<std_msgs::Bool> toggle_inport;
	InputPort<doubles> torque_inport;
	OutputPort<std_msgs::Bool> result_outport;

	std_msgs::Bool toggle;
	bool toggled;
	doubles torques;
	double lowerthreshold;
	double upperthreshold;
	bool lowerthresholdreached;
	bool upperthresholdreached;
	std_msgs::Bool HandoverDetected;
	double threshold;

    public:

    HandoverDetector(const string& name);
    ~HandoverDetector();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
