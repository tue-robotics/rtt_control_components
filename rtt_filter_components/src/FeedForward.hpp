/** FeedForward.hpp
 *
 * @class FeedForward
 *
 * \author Ton Peters
 * \date September, 2014
 * \version 1.0
 *
 */

#ifndef FEEDFORWARD_HPP
#define FEEDFORWARD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;
typedef vector<int> ints;
typedef vector<string> strings;

/**
   * @brief A Component containing a complete feed forward
   * consisting of coulomb and viscous friction compensation,
   * acceleration compensation and a direction dependant
   * friction compensation.
   *
   * Inputs		- Reference velocity
   * 			- Reference acceleration
   * Outputs	- Feed forward output
   *              Safety
   */

class FeedForward
        : public RTT::TaskContext
{
private:

    // Ports
    InputPort<doubles> inport_velocity;
    InputPort<doubles> inport_acceleration;
    OutputPort<doubles> outport_feedforward;
    OutputPort<bool> outport_safety;

    // Properties
    doubles coulomb_gain;
    doubles viscous_gain;
    doubles acceleration_gain;
    doubles direction_gain;
    doubles motor_saturation;
    uint vector_size;

    // Variables
    bool error;

    // Constants
    doubles zero_output;

public:

    FeedForward(const string& name);
    ~FeedForward();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
