/** AdmittanceControllers.hpp
 *
 * @class AdmittanceControllers
 *
 * \author Janno Lunenburg
 * \date August, 2013
 * \version 1.0
 *
 */

#ifndef ADMITTANCECONTROLLERS_HPP
#define ADMITTANCECONTROLLERS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

#include <scl/filters/DFirstOrderLowpass.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{

// Define a new type for easy coding:
typedef vector<double> doubles;

/**
   * @brief A Component that acts as an admittance controller.
   *
   * Based on a virtual masses M and damper constants D, it computes a desired velocity v_r
   * for the input force F
   * using the model M (dv_r/dt) + D v_r = F.
   * This is equal to a first order lowpass filter with gain = 1/D and pole at D/M
   *
   * Finally, it computes the new position reference using x_r = x + v_r *dt
   *
   * @param * M virtual mass or inertia
   *        * D virtual damping coefficient
   */

class AdmittanceControllers
        : public RTT::TaskContext
{
private:

    /* Declaring input and output ports*/
    InputPort<doubles> position_inport;
    InputPort<doubles> force_inport;
    OutputPort<doubles> outport;

    /* Declaring global variables */
    // Variables for history storage
    doubles force_input;
    doubles position_input;
    doubles position_output;
    doubles previous_position_output;
    long double timestamp_last_force_input;

    //doubles previous_output;
    //doubles previous_input;
    //doubles second_previous_output;
    //doubles second_previous_input;

    // Numerator and denominator of the filter
    //doubles a0;
    //doubles a1;
    //doubles a2;
    //doubles b0;
    //doubles b1;
    //doubles b2;

    /* Declaring variables set by properties */
    // Filter parameters
    doubles M;
    doubles D;
    doubles fp;
    doubles k;
    uint vector_size;
    double Ts;

    // Joint limits
    doubles lower_bound;
    doubles upper_bound;

    vector<DFILTERS::DFirstOrderLowpass*> filters;

public:

    AdmittanceControllers(const string& name);
    ~AdmittanceControllers();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
