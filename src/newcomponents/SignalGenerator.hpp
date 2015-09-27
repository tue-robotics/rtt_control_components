#ifndef SIGNALGENERATOR_HPP
#define SIGNALGENERATOR_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_PORTS 10 /* maximum number of ports */
#define PI 3.14159265358979

/*
 * Description:
 * 
 * This component can be used to generate signals. Three formats are
 * distuinguished: Analog (Float), Integer (int) and Digital (bool). The 
 * outputs are vectors of which each individuel output can be controlled.
 * 
 * The output can either be a direct output or packed inside a soem 
 * beckhoff message. Analog (AnalogMsg), Integer (EncoderMsg), Digital
 * (DigitalMsg). Note that an integer output can only be a vector 
 * if it is not inside the  EncoderMsg.
 * 
 * To use this component
 * 1) Start it normally in .ops script without setting any properties
 * 2) Use the functions AddAnalogSignal, AddIntegerSignal, AddDigitalSignal
 *    to add outputs
 * 3) Use the functions below to shape the output by adding base signals
 *    to them. If none of these are called the output is the constant 
 *    default value.
 * 
 * Ana: AddRamp_A,AddNoise_A,AddSine_A,AddStep_A,(AddRefGen_A,AddPulse_A,AddSawTooth_A)
 * Dig: AddStep_D,(AddPulse_D)		
 * Int: AddRamp_I,AddSine_I,AddStep_I,(AddRefGen_I,AddPulse_I,AddSawTooth_I)
 * 
 * Extra Feature:
 * 
 * Add a saturation
 * 
 * To Do:
 * 
 * Add ReferenceGenerator (For Analog and Integer)
 * Add Pulse (For Analog, Integer and Digital)
 * Extensively test the output with tracing component and matlab
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
		
		//! Global
		double TS;
		long double start_time;

		// functions
		virtual void SetOutputZero();
		virtual void WriteOutput();
		virtual void AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ANALOG_MESSAGE);
		virtual void AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool DIGITAL_MESSAGE);
		virtual void AddIntegerSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, bool ENCODER_MESSAGE);

		//! Analog
		// Ports
		OutputPort<doubles> outports_A[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A_msg[MAX_PORTS];
		
		// General
		uint n_analog_signal;
		bool analog_message[MAX_PORTS];
		doubles output_nonadditive_A[MAX_PORTS];
		doubles output_additive_A[MAX_PORTS];
		doubles output_A[MAX_PORTS];
		soem_beckhoff_drivers::AnalogMsg output_A_msgs[MAX_PORTS];

		// Source
		bool ramp_status_A[MAX_PORTS];
		bool sine_status_A[MAX_PORTS];
		bool noise_status_A[MAX_PORTS];
		bool step_status_A[MAX_PORTS];
		doubles ramp_slope_A[MAX_PORTS];
		doubles ramp_endvalue_A[MAX_PORTS];
		doubles sine_amplitude_A[MAX_PORTS];
		doubles sine_frequency_A[MAX_PORTS];
		doubles sine_phase_A[MAX_PORTS];
		doubles noise_mean_A[MAX_PORTS];
		doubles noise_variance_A[MAX_PORTS];
		doubles step_time_A[MAX_PORTS];
		doubles step_value_A[MAX_PORTS];
			
		// Functions
		virtual void AddRamp_A(int ID, doubles SLOPE, doubles FINALVALUE);
		virtual void AddNoise_A(int ID, doubles MEAN, doubles VARIANCE);
		virtual void AddSine_A(int ID, doubles AMPLITUDE, doubles FREQUENCY, doubles PHASE);
		virtual void AddStep_A(int ID, doubles STEPTIME, doubles STEPVALUE);
		virtual void CalculateRamp_A();
		virtual void CalculateNoise_A();
		virtual void CalculateSine_A();
		virtual void CalculateStep_A();
		
		
		//! Integer
		// Ports
		OutputPort<ints> outports_I[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::EncoderMsg> outports_I_msg[MAX_PORTS];
		
		// General
		uint n_integer_signal;
		bool integer_message[MAX_PORTS];
		doubles output_nonadditive_I[MAX_PORTS];
		doubles output_additive_I[MAX_PORTS];
		ints output_I[MAX_PORTS];
		soem_beckhoff_drivers::EncoderMsg output_I_msgs[MAX_PORTS];
		
		// Source
		bool ramp_status_I[MAX_PORTS];
		bool sine_status_I[MAX_PORTS];
		bool noise_status_I[MAX_PORTS];
		bool step_status_I[MAX_PORTS];
		doubles ramp_slope_I[MAX_PORTS];
		doubles ramp_endvalue_I[MAX_PORTS];
		doubles sine_amplitude_I[MAX_PORTS];
		doubles sine_frequency_I[MAX_PORTS];
		doubles sine_phase_I[MAX_PORTS];
		doubles noise_mean_I[MAX_PORTS];
		doubles noise_variance_I[MAX_PORTS];
		doubles step_time_I[MAX_PORTS];
		doubles step_value_I[MAX_PORTS];
			
		// Functions
		virtual void AddRamp_I(int ID, doubles SLOPE, doubles FINALVALUE);
		virtual void AddSine_I(int ID, doubles AMPLITUDE, doubles FREQUENCY, doubles PHASE);
		virtual void AddStep_I(int ID, doubles STEPTIME, doubles STEPVALUE);
		virtual void CalculateRamp_I();
		virtual void CalculateSine_I();
		virtual void CalculateStep_I();
		
		//! Digital
		// Ports
		OutputPort<bool> outports_D[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outports_D_msg[MAX_PORTS];
		
		// General
		uint n_digital_signal;
		bool digital_message[MAX_PORTS];
		doubles output_additive_D[MAX_PORTS];
		bool output_D[MAX_PORTS];
		soem_beckhoff_drivers::DigitalMsg output_D_msgs[MAX_PORTS];
		
		// Source
		bool step_status_D[MAX_PORTS];
		doubles step_time_D[MAX_PORTS];
		doubles step_value_D[MAX_PORTS];
				
		// Function
		virtual void AddStep_D(int ID, doubles STEPTIME, doubles STEPVALUE);
		virtual void CalculateStep_D();
		
		
		//! Support functions
		double randomnumgen(double LOW, double HIGH);
		double box_muller(double M, double V);
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		SignalGenerator(const string& name);
		~SignalGenerator();
	};
}
#endif
