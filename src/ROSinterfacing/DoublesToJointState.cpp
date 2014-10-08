#include <rtt/Component.hpp>

#include "DoublesToJointState.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

DoublesToJointState::DoublesToJointState(const string& name) :
    TaskContext(name, PreOperational)
{
    // ToDo: does this belong here or in the constructor?
    addPort( "pos_in", position_inport_ );
    addPort( "vel_in", velocity_inport_ );
    addPort( "eff_in", effort_inport_ );
    addPort( "out", outport_ );

    addProperty( "JointNames", out_msg_.name );
}

DoublesToJointState::~DoublesToJointState(){}

bool DoublesToJointState::configureHook()
{
	Logger::In in("DoublesToJointState::Configure");
	
    Ndouble_ = out_msg_.name.size();
    log(Info)<<"Size of arrays = "<<Ndouble_<<endlog();
    out_msg_.position.assign(Ndouble_, 0.0);
    out_msg_.velocity.assign(Ndouble_, 0.0);
    out_msg_.effort.assign(Ndouble_, 0.0);
    return true;
}

bool DoublesToJointState::startHook()
{
	Logger::In in("DoublesToJointState::Start");
	
    /// Check which ports are connected
    if (!position_inport_.connected()) {
        log(Warning)<<"WriteJointState: Position inport not connected"<<endlog();
    }
    if (!velocity_inport_.connected()) {
        log(Info)<<"WriteJointState: Velocity inport not connected"<<endlog();
    }
    if (!effort_inport_.connected()) {
        log(Info)<<"WriteJointState: Effort inport not connected"<<endlog();
    }
    if (!outport_.connected()) {
        log(Warning)<<"WriteJointState: Outport not connected"<<endlog();
    }

    return true;
}

void DoublesToJointState::updateHook()
{
	Logger::In in("DoublesToJointState::Update");	
	
    // ToDo: how can we do this nicely?
    doubles pos, vel, eff;
    if ( position_inport_.read( pos ) == NewData)
    {
        for (uint i = 0; i < Ndouble_; i++)
        {
            out_msg_.position[i] = pos[i];
        }
    }
    out_msg_.header.stamp = ros::Time::now();
    outport_.write(out_msg_);

}

ORO_CREATE_COMPONENT(ROS::DoublesToJointState)
