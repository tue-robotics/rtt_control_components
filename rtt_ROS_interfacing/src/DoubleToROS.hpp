/** DoubleToRos.hpp
 *
 * @class DoubleToRos
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

#define maxN 40 //Maximum  size. Still a workaround.

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

  class DoubleToRos
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

    DoubleToRos(const string& name);
    ~DoubleToRos();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
