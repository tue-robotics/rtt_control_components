/** DoubleToROS.hpp
 *
 * @class DoubleToROS
 *
 * \author Tim Clephas
 * \date Sept, 2011
 * \version 1.0
 *
 */

#ifndef DOUBLETOROS_HPP
#define DOUBLETOROS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>

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

  class DoubleToROS
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output port*/
    OutputPort<std_msgs::Float64> doubleoutports[maxN];

    InputPort<double> doubleinports[maxN];


    /* Declaring global variables */
    uint Ndouble; // Number of booleans to output


    public:

    DoubleToROS(const string& name);
    ~DoubleToROS();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
