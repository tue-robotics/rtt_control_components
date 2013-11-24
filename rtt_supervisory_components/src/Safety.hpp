#ifndef SAFETY_HPP
#define SAFETY_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace AMIGO // Just because it looks nice
{
  typedef vector<double> doubles;
  typedef vector<int> ints;

/*! \class Safety
 *  \brief Defines Orocos component for monitoring controllers
 *
 * The Supervisor component monitors:
 *
 * 	* Emergency button state -> if pressed then the interpolators
 * 	  are continuously reset at the measured angle.
 *
 * 	* Controller error -> if too large the amplifiers are disabled.
 *
 * 	* Controller saturation -> if the controller motorspace output
 * 	  is not higher than MOTORSAT for MAXCONSATTIME seconds.
 */

  class Safety
  : public RTT::TaskContext
    {
    private:

        InputPort<doubles> jointErrors_inport;
        InputPort<std_msgs::Bool> eButton_inport;
        InputPort<doubles> controllerOutput_inport;
        InputPort<doubles> mRelJntAng_inport;
        InputPort<uint> homingStatus_inport;

        OutputPort<std_msgs::UInt8> peraStatus_outport;
        OutputPort<bool> enable_outport;
        OutputPort<doubles> resetInt_outport;
        OutputPort<sensor_msgs::JointState> resetRef_outport;

        doubles MAX_ERRORS;
        doubles MOTORSAT;
        doubles timeReachedSaturation;
        doubles jointErrors;
        std_msgs::Bool eButtonPressed;
        bool pressed;
        bool enable;
        bool errors;
        bool homed;
        double MAXCONSATTIME;
        uint jntNr;
        int firstSatInstance [8];
        sensor_msgs::JointState out_msg;
        std_msgs::UInt8 statusToDashboard;

    public:
    
    Safety(const string& name);
    ~Safety();

    bool configureHook();
    bool startHook();
    void updateHook();
    };
}
#endif
