#ifndef SAFETY_HPP
#define SAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#define maxN 10 // maximum number of inputs and outputs

using namespace std;
using namespace RTT;

namespace SAFETY
{
    typedef vector<double> doubles;
    typedef vector<int> ints;
    typedef vector<bool> bools;
    typedef vector<string> strings;

    /*! \class Safety
     *  \brief Defines Orocos component for monitoring controllers
     *
     * The Safety component monitors:
     *
     * 	* Controller error -> if too large the amplifiers are disabled.
     *
     * 	* Controller saturation -> if the controller motorspace output
     * 	  is not higher than MOTORSAT for MAXCONSATTIME seconds.
     *
     *  For a controller in joint space, the error check is performed in joint space
     *  However, the motor saturation check in motor space
     *
     *  Safety sends out safe boolean if no errors are detected.
     *
     *  While Homing, the errors can be increased momentarily to avoid
     *  error state when homing against endstops
     */

    class Safety
            : public RTT::TaskContext
    {
    private:
        // ports
        InputPort<doubles> jointErrors_inport;
        InputPort<doubles> controleffort_inport;
        InputPort<std_msgs::Bool> addsafety_inports[maxN];
        OutputPort<bool> enable_outport;
        OutputPort<bool> error_outport;

        // properties
        uint Nj;
        uint Nm;
        uint partNr;
        double maxConSatTime;
        string prefix;
        doubles maxErrors;
        doubles motorSat;
        strings add_safety_portnames;
        ints errorcntrs;

        // variables
        bool errors;
        doubles jointErrors;
        doubles timeReachedSaturation;
        ints firstSatInstance;

        // TaskContext pointers to peers
        TaskContext* TrajectoryActionlib;

        // Operations
        OperationCaller<void(int)> ResetReferences;

    public:

        Safety(const string& name);
        ~Safety();

        bool configureHook();
        bool startHook();
        void updateHook();
        void stopHook();

    };
}
#endif
