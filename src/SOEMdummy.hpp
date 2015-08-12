#ifndef SOEMDUMMY_HPP
#define SOEMDUMMY_HPP

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
 * This component can be used as a dummy for SOEM. When testing hardware
 * stuff. This component will simulate ethercat slaves. For now these 
 * are have fixed outputs which can be changed via component properties. 
 * 
 * Maybe this can be extended to do an actual
 * hardware simulation to test orocos software.
 * 
 * AnalogSink (controller output)
 * DigitalSink (Brake/Amp enablers)
 * 
 * AnalogSignal (BatteryStatus, TorqueSensor, AbsoluteSensor)
 * DigitalSignal (Fuses/Runstops)
 * IntegerSignal (Encoder)
 * 
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace SOEMDUMMY
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class SOEMdummy
        : public RTT::TaskContext
	{
		public:
		
		//! FUNCTIONS:
		virtual void AddAnalogSink(uint VECTOR_SIZE, string NAME);
		virtual void AddDigitalSink(uint VECTOR_SIZE, string NAME);		
		virtual void AddAnalogSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME);
		virtual void AddDigitalSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME);
		virtual void AddEncoderSignal(uint VECTOR_SIZE, doubles DEFAULT_VALUES, string NAME);		
		virtual void WriteOutput();
		
		//! SINKS:
		
		// ANALOG
		uint n_analog_sink;
		
		// DIGITAL
		uint n_digital_sink;
		
		//! SIGNALS:
		
		// ANALOG
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A_msg[MAX_PORTS];
		uint n_analog_signal;
		doubles output_A[MAX_PORTS];
		soem_beckhoff_drivers::AnalogMsg output_A_msgs[MAX_PORTS];
		
		// DIGITAL
		uint n_digital_signal;
		
		// ENCODER
		uint n_encoder_signal;
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		SOEMdummy(const string& name);
		~SOEMdummy();
	};
}
#endif
