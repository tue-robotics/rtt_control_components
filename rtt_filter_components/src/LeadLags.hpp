/** LeadLags.hpp
 *
 * @class LeadLags
 *
 * \author Boris Mrkajic, Janno Lunenburg
 * \date August, 2013
 * \version 2.0
 *
 */

#ifndef LEADLAGS_HPP
#define LEADLAGS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DLeadLag.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{

// Define a new type for easy coding:
typedef vector<double> doubles;

/**
   * @brief A Component that acts as a lead-lag filters for multiple
   *        inputs
   *
   * The component has one input port that should receive vector of
   * doubles.
   *
   * @param * fz[] [Hz] - array of zero frequencies of the filter
   *        * fp[] [Hz] - array of pole frequencies of the filter
   *        * vector_size - size of the array
   *        * Ts - sampling time
   */

class LeadLags
        : public RTT::TaskContext
{
private:

    /* Declaring input and output ports*/
    InputPort<doubles> inport;
    OutputPort<doubles> outport;

    /* Declaring global variables */
    // Vector of pointers to filters
    vector<DFILTERS::DLeadLag*> filters;

    /* Declaring variables set by properties */
    // Filter parameters
    doubles fz;
    doubles fp;
    uint vector_size;
    double Ts;

public:

    LeadLags(const string& name);
    ~LeadLags();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
