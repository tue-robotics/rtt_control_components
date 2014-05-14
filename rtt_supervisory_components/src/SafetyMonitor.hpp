#ifndef SAFETYMONITOR_HPP
#define SAFETYMONITOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace RTT;

namespace SUPERVISORY
{
  typedef vector<double> doubles;
  typedef vector<int> ints;

/*! \class SafetyMonitor
 *  \brief Defines Orocos component for monitoring controllers
 *
 * The SafetyMonitor component monitors:
 *
 * 	* Controller error -> if too large the amplifiers are disabled.
 *
 * 	* Controller saturation -> if the controller motorspace output
 * 	  is not higher than MOTORSAT for MAXCONSATTIME seconds.
 *   
 *  * Controller sens out safe boolean if no errors are detected.
 * 
 *  * Joints that are homing at the moment are excluded from the error check.
 */

  class SafetyMonitor
  : public RTT::TaskContext
    {
    private:
		// ports
        InputPort<doubles> jointErrors_inport;
        InputPort<doubles> jointAngles_inport; 
        InputPort<doubles> controllerOutput_inport;
        
        InputPort<double> homing_inport; 
        InputPort<bool> nulling_inport;
        
        OutputPort<bool> safe_outport;
        OutputPort<sensor_msgs::JointState> resetRefPort;
        OutputPort<doubles> resetIntPort; // Kept for backwards compatability
        OutputPort<doubles> resetIntPort2; // Used to reset trajectories
        
        // properties
        doubles MAX_ERRORS;
        doubles MOTORSAT;
        double MAXCONSATTIME;
        uint Nj;
        uint Nm;
        uint partNr;

        // variables
        int cntr1;
        int cntr2;
        int counter;
        bool errors;
        bool reNull;
        double homingjoint;
        doubles jointErrors;
        doubles timeReachedSaturation;
        doubles jointAngles;
        doubles resetdata;
        sensor_msgs::JointState out_msg;
        ints firstSatInstance;
		
    public:
    
    SafetyMonitor(const string& name);
    ~SafetyMonitor();

    bool configureHook();
    bool startHook();
    void updateHook();
    
    void reset();    
    };
}
#endif
