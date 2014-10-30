/** WeakIntegrators.hpp
 *
 * @class WeakIntegrators
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef WEAKINTEGRATORS_HPP
#define WEAKINTEGRATORS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DWeakIntegrator.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;

/**
   * @brief A Component that acts as a weak-integrator filter
   *
   * The component has one input port that should receive scalar.
   *
   * @param * fz [Hz] - zero frequency of the filter
   *        * vector_size [0] - size of input vector
   *        * Ts [0.0 sec] - sampling time
   */

class WeakIntegrators
        : public RTT::TaskContext
{
private:

    /* Declaring input and output ports*/
    InputPort<doubles> inport;
    OutputPort<doubles> outport;

    /* Declaring global variables */
    // Vector with pointers to filters
    vector<DFILTERS::DWeakIntegrator*> filters;

    /* Declaring variables set by properties */
    // Filter parameters
    doubles fz;
    uint vector_size;
    double Ts;

public:

    WeakIntegrators(const string& name);
    ~WeakIntegrators();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
