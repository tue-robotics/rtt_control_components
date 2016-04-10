#ifndef ROSWRITE_HPP
#define ROSWRITE_HPP

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

namespace ROSWRITE
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class ROSwrite
        : public RTT::TaskContext
	{
		public:
	
		//!Generic
		strings bodypart_names;
		bool goodToGO;
		long double aquisition_time;
		long double start_time;
		
		virtual void CheckAllConnections();
		virtual void ReadInputs();
		virtual void WriteOutputs();
		virtual void AddAnalogWrite(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		virtual void AddDigitalWrite(string PARTNAME, doubles INPORT_DIMENSIONS, doubles OUTPORT_DIMENSIONS, doubles FROM_WHICH_INPORT, doubles FROM_WHICH_ENTRY);
		
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		ROSwrite(const string& name);
		~ROSwrite();
	};
}
#endif
