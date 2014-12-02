/** SmallerLarger.hpp
 *
 * @class SmallerLarger
 *
 * \author Ton Peters
 * \date Oct, 2014
 * \version 1.0
 *
 */

#ifndef SmallerLarger_HPP
#define SmallerLarger_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

#define maxN 40 //Maximum  size

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace MATH
{
  /*! \class SmallerLarger
   * @brief Check if an input signal is smaller/larger than a constant value
   *
   *
   * @param * numberofinports   - number of input ports
   *        * input_sizes       - size of each input port
   *        * numberofoutports  - number of output ports
   *        * output_sizes      - size of each output port
   *        * smaller           - bool that specifies if smaller or larger
   *                              returns true
   *        * bound_values      - value to compare the signal to
   *        * direct_to_ROS [false] - put the output in a ROS message
   *
   * The component can have multiple input ports that each contain a vector of
   * doubles. Each double input is compared to a specified constant
   * (bound_values) and smaller or larger returns true depending on the
   * comparison specified in smaller.
   */

  class SmallerLarger
  : public RTT::TaskContext
    {
    private:

    typedef vector<double> doubles;
    typedef vector<bool>   bools;
    typedef vector<string> strings;
	typedef std::vector<int> ints;

    /* Declaring in and output ports*/
    InputPort<doubles> inports[maxN];
    OutputPort<bool> outports[maxN];
    OutputPort<std_msgs::Bool> outports_toROS[maxN];

    /* Declaring messages */
    vector<doubles> inputdata;
    bools outputdata;
    std_msgs::Bool outputdata_msg;

    /* Declaring parameters variables */
    uint        n_inports; // number of inports
    uint        n_outports; // number of outports
    doubles     input_sizes; // sizes of the inputports
    //doubles     output_sizes; // sizes of the outputports
    doubles     bound_values; // boundary values
    ints       	smaller; // comparison smaller or larger
    bool        direct_to_ROS; // direct outputs to ros

    /* Declaring global variables */

    public:

    SmallerLarger(const string& name);
    ~SmallerLarger();

    bool configureHook();
    bool startHook();
    void updateHook();

    };
}
#endif
