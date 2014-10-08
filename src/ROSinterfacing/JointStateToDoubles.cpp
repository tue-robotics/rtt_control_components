//#include <rtt/TaskContext.hpp>
//#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "JointStateToDoubles.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

JointStateToDoubles::JointStateToDoubles(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "vector_size", Ndouble_ );
    addEventPort( "in", inport_ );
    addPort( "pos_out", position_outport_ );
    addPort( "vel_out", velocity_outport_ );
    addPort( "eff_out", effort_outport_ );
}

JointStateToDoubles::~JointStateToDoubles(){}

bool JointStateToDoubles::configureHook()
{
	Logger::In in("JointStateToDoubles::Configure");		
	
    pos_out_.assign(Ndouble_, 0.0);
    vel_out_.assign(Ndouble_, 0.0);
    eff_out_.assign(Ndouble_, 0.0);
    return true;
}

bool JointStateToDoubles::startHook()
{
	Logger::In in("JointStateToDoubles::Start");	
	
    /// Check which ports are connected
    if (!position_outport_.connected()) {
        log(Warning)<<"ReadJointState: Position outport not connected"<<endlog();
    }
    if (!velocity_outport_.connected()) {
        log(Info)<<"ReadJointState: Velocity outport not connected"<<endlog();
    }
    if (!effort_outport_.connected()) {
        log(Info)<<"ReadJointState: Effort outport not connected"<<endlog();
    }
    if (!inport_.connected()) {
        log(Warning)<<"ReadJointState: Inport not connected"<<endlog();
    }
    log(Info)<<"JointStateToDoubles can only handle jointstate messages with the correct length"<<endlog();
	
	// This step is done to clear old msgs from this port.
	sensor_msgs::JointState in_msg;
	while (inport_.read(in_msg) == NewData) {
		log(Info)<<"JOINTSTATE TO DOUBLES: Cleared msgs from port"<<endlog();
	}
	
	return true;
}

void JointStateToDoubles::updateHook()
{
	Logger::In in("JointStateToDoubles::Update");	
	
    // ToDo: can't we do this any nicer?
    sensor_msgs::JointState in_msg;
    if (inport_.read(in_msg) == NewData) {
        if (in_msg.position.size() == Ndouble_) {
            for (uint i = 0; i < Ndouble_; i++) {
                pos_out_[i] = in_msg.position[i];
            }
            position_outport_.write(pos_out_);
        }
        if (in_msg.velocity.size() == Ndouble_) {
            for (uint i = 0; i < Ndouble_; i++) {
                vel_out_[i] = in_msg.velocity[i];
            }
            velocity_outport_.write(vel_out_);
        }
        if (in_msg.effort.size() == Ndouble_) {
            for (uint i = 0; i < Ndouble_; i++) {
                eff_out_[i] = in_msg.effort[i];
            }
            effort_outport_.write(eff_out_);
        }
    }

    // ToDo: How do we handle the case that not all joints are actuated?

}

ORO_CREATE_COMPONENT(ROS::JointStateToDoubles)
