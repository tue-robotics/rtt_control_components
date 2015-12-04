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
 * Component that can handle all etherCAT outputs for multiple bodyparts.
 * Also this component can perform mathemathical operations on the inputs.
 * 
 * Contains:	For example:
 * AnalogOuts	Sensor inputs
 * DigitalOuts	Ebuttons
 * 
 * Math operations currently supported 
 * (Encouraged to add yourself/request if you need others)
 * 
 * AnalogOuts:		Addition, Multiply, MatrixTransform
 * DigitalOuts: 	-
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
		strings bodypart_names;
		bool goodToGO;
		long double aquisition_time;
		long double start_time;
		
		virtual void ReadInputs();
		virtual void CheckAllConnections();
		virtual void MapInputs2Outputs();
		virtual void Calculate_A();
		virtual void Calculate_D();
		virtual void WriteOutputs();
		virtual void AddAnalogOuts(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddDigitalOuts(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		
		//! AnalogIns
		// Ports
		InputPort<doubles> inports_A[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::AnalogMsg> outports_A[MAX_BODYPARTS][MAX_PORTS];
	
		// In/Output
		std::vector< doubles > input_A[MAX_PORTS];
		std::vector< soem_beckhoff_drivers::AnalogMsg > output_msgs_A[MAX_PORTS];
		
		// Scalars
		uint n_addedbodyparts_A;
		
		// Vectors
		string added_bodyparts_A[MAX_BODYPARTS];
		uint n_inports_A[MAX_BODYPARTS];
		uint n_outports_A[MAX_BODYPARTS];
		
		// Matrices
		bools addition_status_A[MAX_BODYPARTS];
		bools multiply_status_A[MAX_BODYPARTS];
		bools matrixtransform_status_A[MAX_BODYPARTS];
		ints inport_dimensions_A[MAX_BODYPARTS];
		ints outport_dimensions_A[MAX_BODYPARTS];
		ints from_which_inport_A[MAX_BODYPARTS];
		ints from_which_entry_A[MAX_BODYPARTS];
		
		// 3D	
		vector< doubles > addition_values_A[MAX_BODYPARTS];	
		vector< doubles > multiply_values_A[MAX_BODYPARTS];	
		vector< vector< doubles > > matrixtransform_entries_A[MAX_BODYPARTS];

		// Functions
		virtual void AddAddition_A(string PARTNAME, int PORTNR, doubles VALUES);
		virtual void AddMultiply_A(string PARTNAME, int PORTNR, doubles FACTOR);
		virtual void AddMatrixTransform_A(string PARTNAME, int PORTNR, double INPUTSIZE, double OUTPUTSIZE);
		
		//! DigitalIns
		// Ports
		InputPort<ints> inports_D[MAX_BODYPARTS][MAX_PORTS];
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outports_D[MAX_BODYPARTS][MAX_PORTS];

		// In/Output
		std::vector< ints > input_D[MAX_PORTS];
		std::vector< soem_beckhoff_drivers::DigitalMsg > output_msgs_D[MAX_PORTS];
	
		// Scalars
		uint n_addedbodyparts_D;
		
		// Vectors
		string added_bodyparts_D[MAX_BODYPARTS];
		uint n_inports_D[MAX_BODYPARTS];
		uint n_outports_D[MAX_BODYPARTS];
		
		// Matrices
		ints inport_dimensions_D[MAX_BODYPARTS];
		ints outport_dimensions_D[MAX_BODYPARTS];
		ints from_which_inport_D[MAX_BODYPARTS];
		ints from_which_entry_D[MAX_BODYPARTS];		

		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		EtherCATwrite(const string& name);
		~EtherCATwrite();
	};
}
#endif
