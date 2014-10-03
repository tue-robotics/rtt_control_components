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
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace FILTERS
{
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
    * 			- JointErrors
    */

    class Controller
            : public RTT::TaskContext
    {
        private:

        // Ports
        InputPort<doubles> references_inport;
        InputPort<doubles> positions_inport;
        InputPort<bool> enable_inport;
        OutputPort<doubles> controleffort_outport;
        OutputPort<doubles> jointerrors_outport;

        // Properties
        bool WeakIntegrator;
        bool LeadLag;
        bool Notch;
        bool LowPass;
        bool safe;
        uint vector_size;
        double Ts;
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
        strings controllers;

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
