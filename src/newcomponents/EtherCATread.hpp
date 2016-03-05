#ifndef ETHERCATREAD_HPP
#define ETHERCATREAD_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>

#include <soem_beckhoff_drivers/AnalogMsg.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#define MAX_BODYPARTS 6 /* maximum number of ports */
#define MAX_PORTS 10 	/* maximum number of ports */
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
		
		//! Functions
		// Internal
		virtual void ReadInputs();
		virtual void CheckAllConnections();
		virtual void MapInputs2Outputs();
		virtual void Calculate_A();
		virtual void Calculate_D();
		virtual void Calculate_E();
		virtual void WriteOutputs();
		
		// Internal and External - ResetEncoders
		virtual void ResetEncoders(int BPID, int PORTNR, doubles RESETVALUES );
		
		// External - Add Ins
		virtual void AddAnalogIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddDigitalIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddEncoderIns(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		
		// External - Add Math
		virtual void AddAddition_A(string PARTNAME, int PORTNR, doubles ADDVALUES);
		virtual void AddMultiply_A(string PARTNAME, int PORTNR, doubles MULTIPLYFACTOR);
		virtual void AddCompare_A(string PARTNAME, int PORTNR, string COMPARISON, doubles VALUES);
		virtual void AddTorqueSensor_A(string PARTNAME, int PORTNR, doubles COEFFICIENT1, doubles COEFFICIENT2, doubles COEFFICIENT3);
		virtual void AddFlip_D(string PARTNAME, int PORTNR);
		virtual void AddEnc2Si_E(string PARTNAME, int PORTNR, doubles ENCODERBITS, doubles ENC2SI);
		virtual void AddMatrixTransform_E(string PARTNAME, int PORTNR, double INPUTSIZE, double OUTPUTSIZE);
		virtual void AddSaturation_E(string PARTNAME, int PORTNR, doubles SATURATIONMIN, doubles SATURATIONMAX);
		
		// External - Add MsgOut
		virtual void AddMsgOut_A(string PARTNAME, int PORTNR);
		virtual void AddMsgOut_D(string PARTNAME, int PORTNR);
		virtual void AddMsgOut_E(string PARTNAME, int PORTNR, strings JOINT_NAMES);
		
		//! AnalogIns
		// Ports
		InputPort<soem_beckhoff_drivers::AnalogMsg> inports_A[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<doubles> outports_A[MAX_BODYPARTS][MAX_PORTS];
		OutputPort< vector<bool> > outports_A_comp[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<std_msgs::Float32MultiArray> outports_A_msg[MAX_BODYPARTS][MAX_PORTS];
		
		// In/Output
		vector< soem_beckhoff_drivers::AnalogMsg > input_msgs_A[MAX_BODYPARTS];
		vector< doubles > intermediate_A[MAX_BODYPARTS];
		vector< doubles > output_A[MAX_BODYPARTS];
		vector< bools > output_A_comp[MAX_BODYPARTS];
	
		// Scalars
		uint n_addedbodyparts_A;
		
		// Vectors
		string added_bodyparts_A[MAX_BODYPARTS];
		uint n_inports_A[MAX_BODYPARTS];
		uint n_outports_A[MAX_BODYPARTS];
		uint n_outports_A_comp[MAX_BODYPARTS];
		
		// Matrices
		bools addition_status_A[MAX_BODYPARTS];
		bools multiply_status_A[MAX_BODYPARTS];
		bools comparison_status_A[MAX_BODYPARTS];
		bools torquesensor_status_A[MAX_BODYPARTS];
		bools msgout_status_A[MAX_BODYPARTS];
		ints inport_dimensions_A[MAX_BODYPARTS];
		ints outport_dimensions_A[MAX_BODYPARTS];
		ints from_which_inport_A[MAX_BODYPARTS];
		ints from_which_entry_A[MAX_BODYPARTS];
		// 3D
		vector< doubles > addition_values_A[MAX_BODYPARTS];
		vector< doubles > multiply_values_A[MAX_BODYPARTS];
		vector< doubles > comparison_values_A[MAX_BODYPARTS];
		vector< string > comparison_types_A[MAX_BODYPARTS];
		vector< doubles > torquesensor_c1_A[MAX_BODYPARTS];
		vector< doubles > torquesensor_c2_A[MAX_BODYPARTS];
		vector< doubles > torquesensor_c3_A[MAX_BODYPARTS];
		
		//! DigitalIns
		// Ports
		InputPort<soem_beckhoff_drivers::DigitalMsg> inports_D[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<bool> outports_D[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<std_msgs::Bool> outports_D_msg[MAX_BODYPARTS][MAX_PORTS];

		// In/Output
		vector< soem_beckhoff_drivers::DigitalMsg > input_msgs_D[MAX_BODYPARTS];
		vector< bool > intermediate_D[MAX_BODYPARTS];
		vector< bool > output_D[MAX_BODYPARTS];
	
		// Scalars
		uint n_addedbodyparts_D;
		
		// Vectors
		string added_bodyparts_D[MAX_BODYPARTS];
		uint n_inports_D[MAX_BODYPARTS];
		uint n_outports_D[MAX_BODYPARTS];
		
		// Matrices
		bools flip_status_D[MAX_BODYPARTS];
		bools msgout_status_D[MAX_BODYPARTS];
		ints inport_dimensions_D[MAX_BODYPARTS];
		ints outport_dimensions_D[MAX_BODYPARTS];
		ints from_which_inport_D[MAX_BODYPARTS];
		ints from_which_entry_D[MAX_BODYPARTS];
		
		//! EncoderIns
		// Ports
		InputPort<soem_beckhoff_drivers::EncoderMsg> inports_E[MAX_BODYPARTS][MAX_ENCPORTS];
		OutputPort<doubles> outports_E[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<doubles> outports_E_vel[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<doubles> outports_E_sat[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<sensor_msgs::JointState> outports_E_msg[MAX_BODYPARTS][MAX_PORTS];

		// In/Output
		vector< soem_beckhoff_drivers::EncoderMsg > input_msgs_E[MAX_BODYPARTS];
		vector< doubles > intermediate_E[MAX_BODYPARTS];
		vector< doubles > output_E[MAX_BODYPARTS];
		vector< doubles > output_E_vel[MAX_BODYPARTS];
		vector< doubles > output_E_sat[MAX_BODYPARTS];
		sensor_msgs::JointState output_E_msgs[MAX_BODYPARTS][MAX_PORTS]; 
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
		bools saturation_status_E[MAX_BODYPARTS];
		bools msgout_status_E[MAX_BODYPARTS];
		ints inport_dimensions_E[MAX_BODYPARTS];
		ints outport_dimensions_E[MAX_BODYPARTS];
		ints from_which_inport_E[MAX_BODYPARTS];
		ints from_which_entry_E[MAX_BODYPARTS];
		
		vector< longints > encodercntr_E[MAX_BODYPARTS];
		vector< doubles > initpos_E[MAX_BODYPARTS];
		vector< doubles > encoderbits_E[MAX_BODYPARTS];
		vector< doubles > enc2si_values_E[MAX_BODYPARTS];
		vector< vector< doubles > > matrixtransform_entries_E[MAX_BODYPARTS];
		vector< doubles > saturation_minvalues_E[MAX_BODYPARTS];
		vector< doubles > saturation_maxvalues_E[MAX_BODYPARTS];
		
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
