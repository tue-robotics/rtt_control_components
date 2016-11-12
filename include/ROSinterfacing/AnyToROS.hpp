/** AnyToROS.hpp
 *
 * @class AnyToROS
 *
 * \author Max Baeten
 * \date Nov, 2016
 * \version 1.0
 *
 */

#ifndef ANYTOROS_HPP
#define ANYTOROS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define maxPorts 10 // Maximum number of ports

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

  class AnyToROS
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<int> ints;
    typedef vector<string> strings;

	// Doubles2ROS 
	int n_ports_D;
	ints ids_D[maxPorts];
	doubles input_D[maxPorts];

	/* Declaring functions */
	virtual void AddDoublesToROS(int N, ints ids, string portname);

    /* Declaring output ports*/
	InputPort<doubles> inports_D[maxPorts];
    OutputPort<double> outports_D[maxPorts][maxPorts];
    OutputPort<std_msgs::Float32> outports_D_msg[maxPorts][maxPorts];

    public:

    AnyToROS(const string& name);
    ~AnyToROS();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
