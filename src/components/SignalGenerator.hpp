#ifndef SIGNALGENERATOR_HPP
#define SIGNALGENERATOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_PORTS 10 /* maximum number of ports */

/*
 * Description:
 * 
 * This component can be used to generate ramp signals, constant signals
 * sinuses, noise and much more. Feel free to add missing signal 
 * generators
 * 
 * To do add possibility to internally add two sources to output on one port
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace SIGNALGENERATOR
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class SignalGenerator
        : public RTT::TaskContext
	{
		public:
		
		//! Constant Signal
		// Ports
		OutputPort<doubles> outports_A[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A_msg[MAX_PORTS];
		// Output
		vector<doubles> output_A;
		vector<soem_beckhoff_drivers::AnalogMsg> output_A_msgs;
		// global parameters
		uint n_constant_signal;
		bools analog_message;
		
		//! Functions to add inputs
		virtual void AddConstantSignal(uint VECTOR_SIZE, doubles OUTPUT_VALUES, bool ANALOG_MESSAGE);

		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		SignalGenerator(const string& name);
		~SignalGenerator();
	};
}
#endif
