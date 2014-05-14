#ifndef ACTUATORENABLER_HPP
#define ACTUATORENABLER_HPP

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

/*! \class ActuatorEnabler
 *  \brief Defines Orocos component for enabling actuators (with brake)
 *
 * The ActuatorEnabler component monitors:
 *
 * 	* boolean safe value
 *
 * 	* No new safe = true or safe = false 
 * 		-> amplifier is disabled
 * 		-> possible brake is enabled
 */

  class ActuatorEnabler
  : public RTT::TaskContext
    {
    private:
        InputPort<bool> safe_inPort;
        OutputPort<bool> actuatorEnablePort;
                
        bool safe;
        long double TimeLastSafeReceived;
    public:
    
    ActuatorEnabler(const string& name);
    ~ActuatorEnabler();

    bool configureHook();
    bool startHook();
    void updateHook();
    
    };
}
#endif
