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
#include <scl/filters/DSkewedNotch.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;
typedef vector<int> ints;
typedef vector<string> strings;

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
    doubles fz_WeakIntegrator;
    doubles fz_LeadLag;
    doubles fp_LeadLag;
    doubles fz_Notch;
    doubles dz_Notch;
    doubles fp_Notch;
    doubles dp_Notch;
    doubles fp_LowPass;
    doubles dp_LowPass;
    doubles max_errors;
    doubles motor_saturation;
    double max_sat_time;
    bool WeakIntegrator;
    bool LeadLag;
    bool Notch;
    bool LowPass;
    strings controllers;

    // Variables
    bool errors;
    ints firstSatInstance;
    ints firstErrInstance;
    doubles timeReachedSaturation;
    doubles zero_output;
    int cntr;
    int cntr_10hz;

    // Filters
    vector<DFILTERS::DWeakIntegrator*> filters_WeakIntegrator;
    vector<DFILTERS::DLeadLag*> filters_LeadLag;
    vector<DFILTERS::DSecondOrderLowpass*> filters_LowPass;
    vector<DFILTERS::DSkewedNotch*> filters_Notch;

public:

    Controller(const string& name);
    ~Controller();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
