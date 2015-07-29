#ifndef ADINS_HPP
#define ADINS_HPP

#include <vector>
#include <string>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <std_msgs/Bool.h>

#define MAX_PORTS 20 /* maximum number of ports */

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace SOEM
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	
	class ADIns
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
		ints inport_dimensions_A;
		ints outport_dimensions_A;
		ints from_which_inport_A;
		ints from_which_entry_A;
		
		//! DigitalIns
		// Ports
		InputPort<soem_beckhoff_drivers::DigitalMsg> inports_D[MAX_PORTS];
		OutputPort<std_msgs::Bool> outports_D[MAX_PORTS];
    
		// vectors
		bools flip_D;
		uint n_inports_D;
		uint n_outports_D;

		//! Functions to add inputs
		virtual void AddAnalogIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddDigitalIns(int vector_size, doubles flip_out);

		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		ADIns(const string& name);
		~ADIns();
	};
}
#endif
