#include <rtt/Component.hpp>

#include "JointStateAggregator.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

JointStateAggregator::JointStateAggregator(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("addAggregationPort", &JointStateAggregator::addAggregationPort, this, OwnThread)
            .doc("Add an input port to the JointStateAggregator component")
            .arg("port_name","Name of the port to add");
    addOperation("addJointNames", &JointStateAggregator::addJointNames, this, OwnThread)
            .doc("Add joint names to the out_msg of the JointStateAggregator component")
            .arg("joint_names","Vector with strings of the joint names to add");
    
    addProperty( "JointNames", out_msg_.name );
    //addProperty( "NumberOfInPorts", number_inports_ );
    number_inports_ = 0;

    addPort( "out", outport_ );
    
}

JointStateAggregator::~JointStateAggregator(){}

bool JointStateAggregator::configureHook()
{
    
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
        log(Error)<<"JointStateAggregator: Number of inports "<<number_inports_<<" exceeds maximum of "<<maxN<<endlog();
        return false;
    }
	log(Info)<<"JointStateAggregator: Adding port "<<port_name<<endlog();
    addPort( port_name, inports_[number_inports_ - 1] );

    return true;

}

bool JointStateAggregator::addJointNames(const std::vector<std::string>& joint_names)
{
	for (unsigned int i = 0; i < joint_names.size(); i++)
	{
		out_msg_.name.push_back(joint_names[i]);
		out_msg_.position.push_back(0.0);
		out_msg_.velocity.push_back(0.0);
		out_msg_.effort.push_back(0.0);
		joint_name_to_index_[joint_names[i]] = number_joints_;
		++number_joints_;
	}
	return true;
}

ORO_CREATE_COMPONENT(ROS::JointStateAggregator)
