/** DoublesToJointState.hpp
 *
 * @class JointStateAggregator
 *
 * \author Janno Lunenburg
 * \date September, 2013
 * \version 1.0
 *
 */

#ifndef JOINTSTATEAGGREGATOR_HPP
#define JOINTSTATEAGGREGATOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

#define maxN 40 //Maximum  size

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
/**
   * @brief
   *
   * @param No parameters
   */

class JointStateAggregator
        : public RTT::TaskContext
{
private:

    //typedef vector<double> doubles;
    //typedef vector<string> strings;

    /* Declaring and output ports*/
    //std::vector<InputPort<sensor_msgs::JointState> > inports_;
    InputPort<sensor_msgs::JointState> inports_[maxN];
    OutputPort<sensor_msgs::JointState> outport_;

    /* Declaring global variables */
    uint number_inports_; // Number of inports
    uint number_joints_;  // Total number of joints

    /**
      * Output message
      */
    sensor_msgs::JointState out_msg_;

    /**
      * Maps the joint names to indexes
      */
    std::map<std::string, unsigned int> joint_name_to_index_;

public:

    JointStateAggregator(const string& name);
    ~JointStateAggregator();

    bool configureHook();
    bool startHook();
    void updateHook();

    /**
      * Adds an inport to the aggregator
      * @param port_name Name of the port, e.g., "base" or "left_arm"
      */
    virtual bool addAggregationPort(const std::string& port_name);
    
    /**
      * Adds joints to the JointStateAggregator
      * @param Vector with joint_names Names of the joints to add
      */
    virtual bool addJointNames(const std::vector<std::string>& joint_names);

};
}
#endif
