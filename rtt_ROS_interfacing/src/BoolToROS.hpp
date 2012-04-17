/** BoolToROS.hpp
 *
 * @class BoolToROS
 *
 * \author Tim Clephas
 * \date Sept, 2011
 * \version 1.0
 *
 */

#ifndef BOOLTOROS_HPP
#define BOOLTOROS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

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

  class BoolToROS
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<string> strings;

    /* Declaring and output port*/
    OutputPort<std_msgs::Bool> booloutports[maxN];

    InputPort<bool> boolinports[maxN];


    /* Declaring global variables */
    uint Nbool; // Number of booleans to output


    public:

    BoolToROS(const string& name);
    ~BoolToROS();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
