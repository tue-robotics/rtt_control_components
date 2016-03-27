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

#define MAX_REFIN 3 /* maximum number of ports */
#define MAX_FFWIN 3 /* maximum number of ports */

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

namespace FILTERS
{
    typedef vector<double> doubles;
    typedef vector<int> ints;
    typedef vector<string> strings;

    /**
    * @brief A Component containing a complete controller
    * consisting of multiple filters
    *
    * Inputs	- Referenceports
    * 			- Jointpositions
    *           - FFW inputs
    * Outputs	- Controloutput
    * 			- JointErrors
    *
    * The FFW ports are optional and will be added to the controller output.
    * The references can be obtained from different ports when for example
    * one of the joints has a custom reference generator manipulator grippers
    *
    */

    class Controller
            : public RTT::TaskContext
    {
        private:

        // Ports
        InputPort<doubles> positions_inport;
        InputPort<bool> enable_inport;
        InputPort<doubles> references_inports[MAX_REFIN];
        InputPort<doubles> ffw_inports[MAX_FFWIN];
        OutputPort<doubles> controleffort_outport;
        OutputPort<doubles> jointerrors_outport;
        OutputPort<doubles> references_outport;

        // Properties
        uint vector_size;
        uint N_refinports;
        uint N_ffwinports;
        strings controllers;
        double Ts;
        ints inport_sizes;


        // Controller Properties
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

        // Variables
        bool safe;
        bool WeakIntegrator;
        bool LeadLag;
        bool Notch;
        bool LowPass;
        
        // Various
        doubles zero_output;
	    doubles jointErrors;
        doubles output_Gains;
        doubles output_WeakIntegrator;
        doubles output_LeadLag;
        doubles output_Notch;
        doubles output;
        doubles positions;
        doubles references;
		doubles ref_in[MAX_REFIN];
		doubles ffw_input[MAX_FFWIN];

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
        void stopHook();

    };
}
#endif
