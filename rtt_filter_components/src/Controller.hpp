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
//#include <scl/filters/DWeakIntegrator.hpp>
//#include <scl/filters/DSecondOrderLowpass.hpp>
//#include <scl/filters/DLeadLag.hpp>
//#include <scl/filters/DPID.hpp>

using namespace std;
using namespace RTT;

namespace FILTERS
{
// Define a new type for easy coding:
typedef vector<double> doubles;

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

    // ports
    InputPort<doubles> inport_references;
    InputPort<doubles> inport_positions;
    OutputPort<doubles> outport_controloutput;

    // properties
    uint vector_size;
    doubles gains;

public:

    Controller(const string& name);
    ~Controller();

    bool configureHook();
    bool startHook();
    void updateHook();

};
}
#endif
