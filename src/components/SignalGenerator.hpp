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
 * Make it possible to add different signals (sine, ramp, noise, etc.)
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
		
		// Global
		double TS;
		
		//! Functions to add outputs (and to clear one of them) (By default these signals will have a constant output as defined in DEFAULT_VALUES)
		virtual void AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE);
		virtual void AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool DIGITAL_MESSAGE);
		virtual void AddEncoderSignal(double DEFAULT_VALUE);
		virtual void ClearSignal(string TYPE, int ID);
		
		/*
		 * TYPE SPECIFIC
		*/
		
		//! Analog
		// General
		OutputPort<doubles> outports_A[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A_msg[MAX_PORTS];
		doubles output_A[MAX_PORTS];
		doubles output_property_A[MAX_PORTS];
		soem_beckhoff_drivers::AnalogMsg output_A_msgs[MAX_PORTS];;
		bool analog_message[MAX_PORTS];
		uint n_analog_signal;	
		
		// Source specific
		bool ramp_status[MAX_PORTS];
		doubles ramp_slope[MAX_PORTS];
		doubles ramp_endvalue[MAX_PORTS];
		
		bool noise_status[MAX_PORTS];
		
		bool sine_status[MAX_PORTS];
		
		bool step_status[MAX_PORTS];
		
		// Functions
		virtual void AddRamp(int ID, doubles STARTVALUE, doubles SLOPE, doubles FINALVALUE);
		virtual void AddNoise(int ID, doubles MEAN, doubles VARIANCE);
		virtual void AddSine(int ID, doubles AMPLITUDE, doubles FREQUENCY);
		virtual void AddStep(int ID, doubles STEPTIME, doubles FINALVALUE);
		
		virtual void CalculateRamp();
		virtual void CalculateNoise();
		virtual void CalculateSine();
		virtual void CalculateStep();
	
		
		//! Digital
		OutputPort<ints> outports_D[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outports_D_msg[MAX_PORTS];
		ints output_D[MAX_PORTS];
		doubles output_property_D[MAX_PORTS];
		soem_beckhoff_drivers::DigitalMsg output_D_msgs[MAX_PORTS];
		bool digital_message[MAX_PORTS];
		uint n_digital_signal;
		
		//! Encoder (only possible with encodermsg, otherwise a digital signal can be used)
		OutputPort<soem_beckhoff_drivers::EncoderMsg> outports_E_msg[MAX_PORTS];
		soem_beckhoff_drivers::EncoderMsg output_E_msgs[MAX_PORTS];
		int output_E[MAX_PORTS];
		int output_property_E[MAX_PORTS];
		uint n_encoder_signal;
		



		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		SignalGenerator(const string& name);
		~SignalGenerator();
	};
}
#endif
