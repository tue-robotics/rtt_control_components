#include <rtt/Component.hpp>

#include "JointStateAggregator.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

JointStateAggregator::JointStateAggregator(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "JointNames", out_msg_.name );
    //addProperty( "NumberOfInPorts", number_inports_ );
    number_inports_ = 0;

    addPort( "out", outport_ );
    //inports_.resize(number_inports_);
    /*if ( number_inports_ > maxN )
    {
        log(Error)<<"Number of inports "<<number_inports_<<" exceeds maximum of "<<maxN<<endlog();
    }
    else if ( number_inports_ == 0 )
    {
        log(Error)<<"Please specify the number of inports"<<endlog();
    }

    for (unsigned int i = 0; i < number_inports_; i++ )
    {
        string name_inport = "in"+to_string(i+1);
        addPort( name_inport, inports_[i] );
    }*/
}

JointStateAggregator::~JointStateAggregator(){}

bool JointStateAggregator::configureHook()
{
    /// Properly initialize the out_msgs_;
    number_joints_ = out_msg_.name.size();
    log(Info)<<"Number of joints = "<<number_joints_<<endlog();
    out_msg_.position.assign(number_joints_, 0.0);
    out_msg_.velocity.assign(number_joints_, 0.0);
    out_msg_.effort.assign(number_joints_, 0.0);

    /// Fill the joint_name_to_index_ map
    for (unsigned int i = 0; i < number_joints_; i++)
    {
        joint_name_to_index_[out_msg_.name[i]] = i;
    }
    return true;
}

bool JointStateAggregator::startHook()
{
    /// Check which ports are connected
    for (unsigned int i = 0; i < number_inports_; i++)
    {
        if (!inports_[i].connected())
        {
            log(Warning)<<"Inputport "<<i<<" is not connected"<<endlog();
        }
    }
    if (!outport_.connected())
    {
        log(Warning)<<"WriteJointState: Outport not connected"<<endlog();
    }
    log(Warning)<<"Velocity and effort not yet implemented"<<endlog();
    return true;
}

void JointStateAggregator::updateHook()
{
    sensor_msgs::JointState in_msg;
    /// Loop over input ports
    for (unsigned int i = 0; i < number_inports_; i++ )
    {
        /// Only fill in if there's new data
        if ( inports_[i].read(in_msg) == NewData)
        {
            /// Loop over input message
            for (unsigned int j = 0; j < in_msg.name.size(); j++)
            {
                unsigned int index = joint_name_to_index_[in_msg.name[j]];
                /// Only copy data if contains exists
                if ( !in_msg.position.empty() )
                {
                    out_msg_.position[index] = in_msg.position[j];
                }
                if ( !in_msg.velocity.empty() )
                {
                    out_msg_.velocity[index] = in_msg.velocity[j];
                }
                if ( !in_msg.effort.empty() )
                {
                    out_msg_.effort[index] = in_msg.effort[j];
                }
            }
        }
    }

    out_msg_.header.stamp = ros::Time::now();
    outport_.write(out_msg_);
    //log(Warning)<<"Running!"<<endlog();

}

bool JointStateAggregator::addAggregationPort(const std::string& port_name)
{
    /// Check whether number of inports does not exceed maximum
    ++number_inports_;
    if ( number_inports_ > maxN )
    {
        log(Error)<<"Number of inports "<<number_inports_<<" exceeds maximum of "<<maxN<<endlog();
        return false;
    }

    addPort( port_name, inports_[number_inports_ - 1] );

    return true;

}

ORO_CREATE_COMPONENT(ROS::JointStateAggregator)
