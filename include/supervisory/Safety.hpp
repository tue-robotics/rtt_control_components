#ifndef SAFETY_HPP
#define SAFETY_HPP

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
        OutputPort<bool> safe_outport;
        
        // properties
        uint NJ;
        uint NM;
        doubles MAX_ERRORS;
        doubles MOTORSAT;
        double MAXCONSATTIME;


        // variables
        bool errors;
        doubles jointErrors;
        doubles timeReachedSaturation;
        ints firstSatInstance;


		
    public:
    
    Safety(const string& name);
    ~Safety();

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    

    virtual void SetMaxErrors( doubles SET_MAX_ERRORS );

    };
}
#endif
