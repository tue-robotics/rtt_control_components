/** DoublesToROS.hpp
 *
 * @class DoublesToROS
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
#include <std_msgs/Float32.h>

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

  class DoublesToROS
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output port*/
    OutputPort<std_msgs::Float32> doubleoutports[maxN];

    InputPort<doubles> doubleinport;


    /* Declaring global variables */
    uint Ndouble; // Number of doubles in vector


    public:

    DoublesToROS(const string& name);
    ~DoublesToROS();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
