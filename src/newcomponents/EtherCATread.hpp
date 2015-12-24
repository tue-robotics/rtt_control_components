#ifndef ETHERCATREAD_HPP
#define ETHERCATREAD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

#define MAX_BODYPARTS 6 /* maximum number of ports */
#define MAX_PORTS 5 	/* maximum number of ports */
#define MAX_ENCPORTS 10 /* maximum number of ports */

/*
 * Description:
 * 
 * Component that can handle all etherCAT inputs for multiple bodyparts.
 * Also this component can perform mathemathical operations on the inputs.
 * 
 * Contains:	For example:
 * AnalogIns	Sensor inputs
 * DigitalIns	Ebuttons
 * EncoderIns	Encoders
 * 
 * Math operations currently supported
 * (Encouraged to add yourself/request if you need others)
 * 
 * AnalogIns:	Addition, Multiply
 * DigitalIns: 	Flip
 * Encoders:	Enc2SI, MatrixTransform
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace ETHERCATREAD
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector< long long int > longints;
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
		strings bodypart_names;
		
		// Functions
		virtual void ReadInputs();
		virtual void CheckAllConnections();
		virtual void MapInputs2Outputs();
		virtual void Calculate_A();
		virtual void Calculate_D();
		virtual void Calculate_E();
		virtual void WriteOutputs();
		virtual void AddAnalogIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddDigitalIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddEncoderIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		
		//! AnalogIns
		// Ports
		InputPort<soem_beckhoff_drivers::AnalogMsg> inports_A[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<doubles> outports_A[MAX_BODYPARTS][MAX_PORTS];
		
		// In/Output
		vector< soem_beckhoff_drivers::AnalogMsg > input_msgs_A[MAX_BODYPARTS];
		vector< doubles > intermediate_A[MAX_BODYPARTS];
		vector< doubles > output_A[MAX_BODYPARTS];	
	
		// Scalars
		uint n_addedbodyparts_A;
		
		// Vectors
		string added_bodyparts_A[MAX_BODYPARTS];
		uint n_inports_A[MAX_BODYPARTS];
		uint n_outports_A[MAX_BODYPARTS];;
		
		// Matrices
		bools addition_status_A[MAX_BODYPARTS];
		bools multiply_status_A[MAX_BODYPARTS];
		ints inport_dimensions_A[MAX_BODYPARTS];
		ints outport_dimensions_A[MAX_BODYPARTS];
		ints from_which_inport_A[MAX_BODYPARTS];
		ints from_which_entry_A[MAX_BODYPARTS];
		// 3D
		vector< doubles > addition_values_A[MAX_BODYPARTS];	
		vector< doubles > multiply_values_A[MAX_BODYPARTS];	

		// Functions
		virtual void AddAddition_A(string PARTNAME, int PORTNR, doubles ADDVALUES);
		virtual void AddMultiply_A(string PARTNAME, int PORTNR, doubles MULTIPLYFACTOR);

		
		//! DigitalIns
		// Ports
		InputPort<soem_beckhoff_drivers::DigitalMsg> inports_D[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<bools> outports_D[MAX_BODYPARTS][MAX_PORTS];

		// In/Output
		vector< soem_beckhoff_drivers::DigitalMsg > input_msgs_D[MAX_BODYPARTS];
		vector< bools > intermediate_D[MAX_BODYPARTS];
		vector< bools > output_D[MAX_BODYPARTS];
	
		// Scalars
		uint n_addedbodyparts_D;
		
		// Vectors
		string added_bodyparts_D[MAX_BODYPARTS];
		uint n_inports_D[MAX_BODYPARTS];
		uint n_outports_D[MAX_BODYPARTS];
		
		// Matrices
		bools flip_status_D[MAX_BODYPARTS];
		ints inport_dimensions_D[MAX_BODYPARTS];
		ints outport_dimensions_D[MAX_BODYPARTS];
		ints from_which_inport_D[MAX_BODYPARTS];
		ints from_which_entry_D[MAX_BODYPARTS];
		// 3D
		vector< bools > flip_values_D[MAX_BODYPARTS];		
		
		// Functions
		virtual void AddFlip_D(string PARTNAME, int PORTNR, doubles FLIPVALUES);
		
		//! EncoderIns
		// Ports
		InputPort<soem_beckhoff_drivers::EncoderMsg> inports_E[MAX_BODYPARTS][MAX_ENCPORTS];
		OutputPort<doubles> outports_E[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<doubles> outports_E_vel[MAX_BODYPARTS][MAX_PORTS];
		
		// In/Output
		vector< soem_beckhoff_drivers::EncoderMsg > input_msgs_E[MAX_BODYPARTS];
		vector< doubles > intermediate_E[MAX_BODYPARTS];
		vector< doubles > output_E[MAX_BODYPARTS];
		vector< doubles > output_E_vel[MAX_BODYPARTS];
		vector< doubles > enc_values[MAX_BODYPARTS];
		vector< doubles > previous_enc_values[MAX_BODYPARTS];
		
		// Scalars
		double old_time;
		uint n_addedbodyparts_E;
		
		// Vectors
		string added_bodyparts_E[MAX_BODYPARTS];
		uint n_inports_E[MAX_BODYPARTS];
		uint n_outports_E[MAX_BODYPARTS];
		
		// Matrices		
		bools enc2si_status_E[MAX_BODYPARTS];
		bools matrixtransform_status_E[MAX_BODYPARTS];
		ints inport_dimensions_E[MAX_BODYPARTS];
		ints outport_dimensions_E[MAX_BODYPARTS];
		ints from_which_inport_E[MAX_BODYPARTS];
		ints from_which_entry_E[MAX_BODYPARTS];
		
		vector< longints > encodercntr_E[MAX_BODYPARTS];
		vector< doubles > initpos_E[MAX_BODYPARTS];
		
		vector< doubles > encoderbits_E[MAX_BODYPARTS];
		vector< doubles > enc2si_values_E[MAX_BODYPARTS];		
		vector< vector< doubles > > matrixtransform_entries_E[MAX_BODYPARTS];
		
		// Functions
		virtual void AddEnc2Si_E(string PARTNAME, int PORTNR, doubles ENCODERBITS, doubles ENC2SI);
		virtual void AddMatrixTransform_E(string PARTNAME, int PORTNR, double INPUTSIZE, double OUTPUTSIZE);
		
		virtual void ResetEncoders(int BPID, int PORTNR, doubles RESETVALUES );
		
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
