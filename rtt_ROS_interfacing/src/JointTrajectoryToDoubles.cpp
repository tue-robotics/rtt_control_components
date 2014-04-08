//#include <rtt/TaskContext.hpp>
//#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "JointTrajectoryToDoubles.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

JointTrajectoryToDoubles::JointTrajectoryToDoubles(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "NumberOfJoints", Ndouble_ );
    addPort( "in", inport_ );
    addPort( "pos_out", position_outport_ );
    addPort( "eff_out", effort_outport_ );
}

JointTrajectoryToDoubles::~JointTrajectoryToDoubles(){}

bool JointTrajectoryToDoubles::configureHook()
{
    pos_out_.assign(Ndouble_, 0.0);
    eff_out_.assign(Ndouble_, 0.0);
    return true;
}

bool JointTrajectoryToDoubles::startHook()
{
    /// Check which ports are connected
    if (!position_outport_.connected())
    {
        log(Warning)<<"ReadJointState: Position outport not connected"<<endlog();
    }
    if (!effort_outport_.connected())
    {
        log(Info)<<"ReadJointState: Effort outport not connected"<<endlog();
    }
    if (!inport_.connected())
    {
        log(Warning)<<"ReadJointState: Inport not connected"<<endlog();
    }
    playing_trajectory = false;

    return true;
}

void JointTrajectoryToDoubles::updateHook()
{
    if (inport_.read(in_msg) == NewData)
    {
		tp = 0;
        playing_trajectory = true;
        // TODO: Check first point with last point of previous trajectory. Should match (ish)
        // TODO: Check dt = 0.001
	}
	if (tp > in_msg.points.size())
	{
		playing_trajectory = false;
	}

	if (playing_trajectory)
	{
		trajectory_msgs::JointTrajectoryPoint point_msg;
		point_msg = in_msg.points[tp];

        if (point_msg.positions.size() == Ndouble_)
        {
            for (uint i = 0; i < Ndouble_; i++)
            {
                pos_out_[i] = point_msg.positions[i];
            }
            position_outport_.write(pos_out_);
        }
        if (point_msg.accelerations.size() == Ndouble_)
        {
            for (uint i = 0; i < Ndouble_; i++)
            {
                eff_out_[i] = point_msg.accelerations[i];
            }
            effort_outport_.write(eff_out_);
        }
    }

    // ToDo: How do we handle the case that not all joints are actuated?

}

ORO_CREATE_COMPONENT(ROS::JointTrajectoryToDoubles)
