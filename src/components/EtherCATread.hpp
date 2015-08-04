#ifndef ETHERCATREAD_HPP
#define ETHERCATREAD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_PORTS 20 /* maximum number of ports */

/*
 * Description:
 * 
 * Iterators
 * i Loops over ports
 * j Loops over total number of entries
 * k Loops over the number of entries with a givven port
 *  
 * To Do
 * Test with more complex structures
 * Add standard math operations to inputs (such that directly the measured torque can be outputted, or the motor2jointspace conversion)
 * Add read encoders operation
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace ETHERCATREAD
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class EtherCATread
        : public RTT::TaskContext
	{
		public:
	
		//!Generic
		bool goodToGO;
		long double aquisition_time;
		long double start_time;
		
		//! AnalogIns
		// Ports
		InputPort<soem_beckhoff_drivers::AnalogMsg> inports_A[MAX_PORTS];
		OutputPort<doubles> outports_A[MAX_PORTS];
	
		// Scalars
		uint n_inports_A;
		uint n_outports_A;
		uint n_inport_entries_A;
		uint n_outport_entries_A;
		
		// Vectors
		strings added_bodyparts_A;
		ints inport_dimensions_A;
		ints outport_dimensions_A;
		ints from_which_inport_A;
		ints from_which_entry_A;
		
		// In/Output
		std::vector< soem_beckhoff_drivers::AnalogMsg > input_msgs_A;
		std::vector< doubles > output_A;
		
		//! DigitalIns
		// Ports
		InputPort<soem_beckhoff_drivers::DigitalMsg> inports_D[MAX_PORTS];
		OutputPort<ints> outports_D[MAX_PORTS];
	
		// Scalars
		uint n_inports_D;
		uint n_outports_D;
		uint n_inport_entries_D;
		uint n_outport_entries_D;
		
		// Vectors
		strings added_bodyparts_D;
		ints inport_dimensions_D;
		ints outport_dimensions_D;
		ints from_which_inport_D;
		ints from_which_entry_D;
		
		// In/Output
		std::vector< soem_beckhoff_drivers::DigitalMsg > input_msgs_D;
		std::vector< ints > output_D;

		//! EncoderIns
		// Ports
		InputPort<soem_beckhoff_drivers::EncoderMsg> inports_E[MAX_PORTS];
		OutputPort<ints> outports_E[MAX_PORTS];
	
		// Scalars
		uint n_inports_E;
		uint n_outports_E;
		uint n_inport_entries_E;
		uint n_outport_entries_E;
		
		// Vectors
		strings added_bodyparts_E;
		ints inport_dimensions_E;
		ints outport_dimensions_E;
		ints from_which_inport_E;
		ints from_which_entry_E;
		
		// In/Output
		std::vector< soem_beckhoff_drivers::EncoderMsg > input_msgs_E;
		std::vector< ints > output_E;

		//! Functions to add inputs
		virtual void AddAnalogIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		virtual void AddDigitalIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		virtual void AddEncoderIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);

		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		EtherCATread(const string& name);
		~EtherCATread();
	};
}
#endif
