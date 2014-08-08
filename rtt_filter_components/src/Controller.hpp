/** Controller.hpp
 *
 * @class Controller
 *
 * \author Max Baeten
 * \date August, 2014
 * \version 1.0
 *
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>
#include <scl/filters/DWeakIntegrator.hpp>
#include <scl/filters/DSecondOrderLowpass.hpp>
#include <scl/filters/DLeadLag.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;
typedef vector<int> ints;

/**
   * @brief A Component containing a complete controller
   * consising of multiple filters and a safety mechanism
   *
   * Inputs		- Reference 
   * 			- Encoderpositions
   * Outputs	- Controloutput
   * 			- Enable Signal
   * 			- JointErrors
   */

class Controller
        : public RTT::TaskContext
{
private:

    // Ports
    InputPort<doubles> inport_references;
    InputPort<doubles> inport_positions;
    OutputPort<doubles> outport_controloutput;
    OutputPort<bool> outport_safety;
    OutputPort<doubles> outport_controlerrors;

    // Properties
    double Ts;
    uint vector_size;
    doubles gains;
    doubles fz_WI;
    doubles fz_LL;
    doubles fp_LL;
    doubles fp_LP;
    doubles dp_LP;
    doubles max_errors;
    doubles motor_saturation;
    double max_sat_time;

    // Variables
    bool errors;
    ints firstSatInstance;
    ints firstErrInstance;
    doubles timeReachedSaturation;
    doubles zero_output;
    int cntr;
    int cntr_10hz;

    // Filters
    vector<DFILTERS::DWeakIntegrator*> filters_WI;
    vector<DFILTERS::DLeadLag*> filters_LL;
    vector<DFILTERS::DSecondOrderLowpass*> filters_LP;

public:

    Controller(const string& name);
    ~Controller();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
