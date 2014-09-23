#ifndef SATURATIONCHECK_HPP
#define	SATURATIONCHECK_HPP

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
	typedef std::vector< std::vector<double> > doubless;
	typedef vector<int> ints;
	typedef vector<bool> bools;

	/*! \class SaturationCheck
	*  \brief Defines Orocos component for checking input saturation
	*
	* The SaturationCheck component monitors:
	*
	* 	* Control input
	*
	* 	* No new safe = true or safe = false 
	* 		-> amplifier is disabled
	* 		-> possible brake is enabled
	*/

	class SaturationCheck
	: public RTT::TaskContext
	{
		private:
		
		// Declaration of ports
		InputPort<doubles> control_inPort[maxN];
		OutputPort<bool> enablePort;

		// Declaration of properties
		uint N_inPorts;
		uint N_signals;
		doubles motor_saturation;
		double max_sat_time;
		doubles input_sizes;

		// Declaration of variables
		bool enable;
		doubless inputdata;
		bools firstSatInstance;
		doubles timeReachedSaturation;


		public:
		
		SaturationCheck(const string& name);
		~SaturationCheck();

		bool configureHook();
		bool startHook();
		void updateHook();
	};
}
#endif
