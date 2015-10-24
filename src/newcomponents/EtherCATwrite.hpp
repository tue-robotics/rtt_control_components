#ifndef ETHERCATWRITE_HPP
#define ETHERCATWRITE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_BODYPARTS 10 /* maximum number of ports */
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
 * (Maybe add math, first do read math)
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace ETHERCATWRITE
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class EtherCATwrite
        : public RTT::TaskContext
	{
		public:
	
		//!Generic
		bool goodToGO;
		long double aquisition_time;
		long double start_time;
		
		virtual void ReadInputs();
		virtual void CheckAllConnections();
		virtual void MapInput2Outputs();
		virtual void Calculate_A();
		virtual void Calculate_D();
		virtual void WriteOutputs();
		virtual void AddAnalogOuts(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		virtual void AddDigitalOuts(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		
		//! AnalogIns
		// Ports
		InputPort<doubles> inports_A[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A[MAX_PORTS];
	
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
		std::vector< doubles > input_A;
		std::vector< soem_beckhoff_drivers::AnalogMsg > output_msgs_A;
		
		// Math
		bool addition_status_A[MAX_BODYPARTS];
		bool multiply_status_A[MAX_BODYPARTS];
		bool matrixtransform_status_A[MAX_BODYPARTS];
		doubles addition_values_A[MAX_BODYPARTS];		
		doubles multiply_factor_A[MAX_BODYPARTS];		
		std::vector< soem_beckhoff_drivers::AnalogMsg > input_MT_A;	
		std::vector< doubles > matrixtransform_A[MAX_BODYPARTS];

		// Functions
		virtual void AddAddition_A(int ID, doubles VALUES);
		virtual void AddMultiply_A(int ID, doubles FACTOR);
		virtual void AddMatrixTransform_A(int ID);
		
		//! DigitalIns
		// Ports
		InputPort<ints> inports_D[MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outports_D[MAX_PORTS];
	
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
		std::vector< ints > input_D;
		std::vector< soem_beckhoff_drivers::DigitalMsg > output_msgs_D;

		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		EtherCATwrite(const string& name);
		~EtherCATwrite();
	};
}
#endif