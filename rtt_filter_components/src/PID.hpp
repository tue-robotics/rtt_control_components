/** PID.hpp
 *
 * @class PID
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef PID_HPP
#define PID_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DPID.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;

/**
   * @brief A Component that acts as a PID filter with anti-windup
   *
   * The component has one input port that should receive array.
   *
   * @param * kp [-] - proportional gain
   *        * kv [-] - derivative gain
   *        * ki [-] - integral gain
   *        * kaw [-] - anti-windup gain
   *        * init [-] - initial value of the integrator
   *        * limit [-] - output limit (saturation value)
   *        * Ts [0.0 sec] - sampling time
   *        * vector_size [0] - size of input vector
   */

class PID
        : public RTT::TaskContext
{
private:

    /* Declaring input and output ports*/
    InputPort<doubles> inport;
    OutputPort<doubles> outport;

    /* Declaring global variables */
    // Vector of pointers to filters
    vector<DFILTERS::DPID*> filters;

    /* Declaring variables set by properties */
    // Filter parameters
    doubles kp;
    doubles kv;
    doubles ki;
    doubles kaw;
    doubles init;
    doubles limit;
    double Ts;
    uint vector_size;

public:

    PID(const string& name);
    ~PID();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
