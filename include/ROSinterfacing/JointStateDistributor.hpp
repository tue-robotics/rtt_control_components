/** JointStateDistributor.hpp
 *
 * @class DoublesToJointState
 *
 * \author Max Baeten
 * \date Dec, 2014
 * \version 1.0
 *
 */

#ifndef JOINTSTATEDISTRIBUTOR_HPP
#define JOINTSTATEDISTRIBUTOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <sensor_msgs/JointState.h>

#define maxN 8 

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace ROS
{
  /**
   * @brief
   *
   * @param No parameters
   */

  class JointStateDistributor
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<int> ints;
    typedef vector<string> strings;

    InputPort<sensor_msgs::JointState> inport;
    OutputPort<doubles> pos_outport[maxN];
    OutputPort<doubles> vel_outport[maxN];
    OutputPort<doubles> eff_outport[maxN];

    doubles pos_out[maxN];
    doubles vel_out[maxN];
    doubles eff_out[maxN];
    ints activeBodyparts;
    int totalNumberOfJoints;
    int numberOfBodyparts;
    bool ConnectionsChecked;
    
    public:

    JointStateDistributor(const string& name);
    ~JointStateDistributor();

    bool configureHook();
    bool startHook();
    void updateHook();
    void AddBodyPart(int partNr, strings JointNames);
    
    typedef pair<int, int> BodyJointPair;
    map<string, BodyJointPair> joint_map;
    };
}
#endif
