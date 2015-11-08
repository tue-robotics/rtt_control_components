#ifndef ETHERCATREAD_HPP
#define ETHERCATREAD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_BODYPARTS 10 /* maximum number of ports */
#define MAX_PORTS 20 /* maximum number of ports */
#define MAX_ENCPORTS 40 /* maximum number of ports */

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
 * Add math:
 * - SensorTorques
 * - Matrix Transforms
 * - Multiply
 * - ReadEncoders
 * 
 * 
 * 
 * Iterators
 * 
 * i  loops over in/outports
 * j  loops over entries 
 * k  loops 
 * l  loops over bodyparts
 * 
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
		
		// Functions
		virtual void ReadInputs();
		virtual void CheckAllConnections();
		virtual void MapInput2Outputs();
		virtual void Calculate_A();
		virtual void Calculate_D();
		virtual void Calculate_E();
		virtual void WriteOutputs();
		virtual void AddAnalogIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		virtual void AddDigitalIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);
		virtual void AddEncoderIns(doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY, string PARTNAME);	
		
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
		std::vector< doubles > intermediate_A;
		
		// Math
		bool addition_status_A[MAX_BODYPARTS];
		bool multiply_status_A[MAX_BODYPARTS];
		doubles addition_values_A[MAX_BODYPARTS];		
		doubles multiply_factor_A[MAX_BODYPARTS];	

		// Functions
		virtual void AddAddition_A(int ID, doubles VALUES);
		virtual void AddMultiply_A(int ID, doubles FACTOR);

		
		//! DigitalIns
		// Ports
		InputPort<soem_beckhoff_drivers::DigitalMsg> inports_D[MAX_PORTS];
		OutputPort<bools> outports_D[MAX_PORTS];
	
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
		std::vector< bools > output_D;
		std::vector< bools > intermediate_D;

		// Math
		bool flip_status_D[MAX_BODYPARTS];
		bools flip_flip_D[MAX_BODYPARTS];		

		// Functions
		virtual void AddFlip_D(int ID, doubles FLIP);
		
		//! EncoderIns
		// Ports
		InputPort<soem_beckhoff_drivers::EncoderMsg> inports_E[MAX_ENCPORTS];
		OutputPort<doubles> outports_E[MAX_PORTS];
		OutputPort<doubles> outports_E_vel[MAX_PORTS];
		
		// Scalars
		uint n_inports_E;
		uint n_outports_E;
		uint n_inport_entries_E;
		uint n_outport_entries_E;
		double old_time;
		
		// Vectors
		strings added_bodyparts_E;
		ints inport_dimensions_E;
		ints outport_dimensions_E;
		ints from_which_inport_E;
		ints from_which_entry_E;
		std::vector< long long int > ienc[MAX_BODYPARTS];
		doubles encoderbits[MAX_BODYPARTS];
		doubles enc2SI[MAX_BODYPARTS];
		doubles position_SI_init[MAX_BODYPARTS];
		doubles enc_values[MAX_BODYPARTS];
		doubles previous_enc_values[MAX_BODYPARTS];
		
		// In/Output
		std::vector< soem_beckhoff_drivers::EncoderMsg > input_msgs_E;
		std::vector< doubles > output_E;
		std::vector< doubles > intermediate_E;
		std::vector< doubles > output_E_vel;

		// Math
		bool enc2si_status_E[MAX_BODYPARTS];
		bool matrixtransform_status_E[MAX_BODYPARTS];
		std::vector< doubles > input_MT_E;
		std::vector< doubles > input_MT_E_vel;		
		std::vector< doubles > matrixtransform_E[MAX_BODYPARTS];
		
		// Functions
		virtual void AddEnc2Si_E(int ID, doubles ENCODERBITS, doubles ENC2SI);
		virtual void AddMatrixTransform_E(int ID);
		virtual void ResetEncoders(int ID, doubles resetvalues );
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		EtherCATread(const string& name);
		~EtherCATread();
		
		
		
		private:
		
		double determineDt();
		
	};
}
#endif
