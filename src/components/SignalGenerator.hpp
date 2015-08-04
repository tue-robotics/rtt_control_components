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
 * To do add parameters as is normally done which will make the setValue function obsolete
 * To do add math functions such as setSine
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
		
		//! Analog
		OutputPort<doubles> outports_A[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A_msg[MAX_PORTS];
		vector<doubles> output_A;
		vector<doubles> output_property_A;
		vector<soem_beckhoff_drivers::AnalogMsg> output_A_msgs;
		uint n_analog_signal;
		bools analog_message;		
		
		//! Digital
		OutputPort<ints> outports_D[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outports_D_msg[MAX_PORTS];
		vector<ints> output_D;
		vector<doubles> output_property_D;
		vector<soem_beckhoff_drivers::DigitalMsg> output_D_msgs;
		uint n_digital_signal;
		bools digital_message;
		
		//! Encoder (only possible with encodermsg, otherwise a digital signal can be used)
		OutputPort<soem_beckhoff_drivers::EncoderMsg> outports_E_msg[MAX_PORTS];
		vector<soem_beckhoff_drivers::EncoderMsg> output_E_msgs;
		ints output_E;
		ints output_property_E;
		uint n_encoder_signal;
		bools encoder_message;
		
		//! Functions to add inputs (By default these signals will have a constant output as defined in DEFAULT_VALUES)
		virtual void AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE);
		virtual void AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool DIGITAL_MESSAGE);
		virtual void AddEncoderSignal(double DEFAULT_VALUE);
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		SignalGenerator(const string& name);
		~SignalGenerator();
	};
}
#endif
