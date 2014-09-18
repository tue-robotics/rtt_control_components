#ifndef ACTUATORENABLER_HPP
#define ACTUATORENABLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#define maxN 5 // maximum number of inputs and outputs,

using namespace std;

template <class T>
inline string to_string (const T& t){
  stringstream ss;
  ss << t;
  return ss.str();
};

using namespace RTT;

namespace SUPERVISORY
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;

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
		
		InputPort<bool> safe_inPort[maxN];
		OutputPort<bool> actuatorEnablePort;

		uint N_safeIn;
		bools safe;
		bool ErrorWritten;
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
