/** Current2VoltageFFW.hpp
 *
 * @class Current2VoltageFFW
 *
 * \author Max Baeten
 * \date September, 2014
 * \version 1.0
 *
 */

#ifndef CURRENT2VOLTAGEFFW_HPP
#define CURRENT2VOLTAGEFFW_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

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

	/** @brief A Component that calculates pwm values to realise a desired 
	torque. 

	1) The torque inputs of all inports[i] are added
	2) Torques converted into a current
	3) Derivatives of the current and position are then determined 
	4) Desired voltage is calculated via Va = Ke*Wm + Ra*Ia + La*dIa/dt
	5) Voltage is converted into a PWM value
	*/

	class Current2VoltageFFW
	: public RTT::TaskContext
	{
		private:
			// Input and output ports
			InputPort<doubles> inport_position;
			InputPort<doubles> inport_current;
			OutputPort<doubles> outport;

			// global
			doubles previous_output;
			doubles previous_input;
			long double dt;
			long double old_time;
			std::vector<double> a;
			std::vector<double> b;

			// Properties
			uint N;
			double Ts;
			doubles Ke;
			doubles GR;
			doubles Ra;
			doubles Volt2PWM;

			// Declaring private functions
			doubles calculatederivative(doubles diff_in);

		public:
			Current2VoltageFFW(const string& name);
			~Current2VoltageFFW();

			bool configureHook();
			bool startHook();
			void updateHook();
    };
}
#endif
