#ifndef COMPARE_HPP
#define COMPARE_HPP

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
 * This component can be used to compare signals.
 * 
*/

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace COMPARE
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	typedef vector<string> strings;
	
	class Compare
        : public RTT::TaskContext
	{
		public:
		
		// DOUBLES:
		virtual void AddCompareDoubles(double SIZE, bool MSG);
		virtual void CompareDoubles();
		
		InputPort<doubles> Ainports_D[MAX_PORTS];
		InputPort<doubles> Binports_D[MAX_PORTS];
		InputPort<soem_beckhoff_drivers::AnalogMsg> Ainports_D_msg[MAX_PORTS];
		InputPort<soem_beckhoff_drivers::AnalogMsg> Binports_D_msg[MAX_PORTS];

		std::vector< doubles > input_A;
		std::vector< doubles > input_B;
		std::vector< soem_beckhoff_drivers::AnalogMsg > input_A_msg;
		std::vector< soem_beckhoff_drivers::AnalogMsg > input_B_msg;
		
		double portsizes_D[MAX_PORTS];
		double warningtime;		
		bool msgsstore[MAX_PORTS];
		int n_comparisons_D;		
		int k_timer;
		bool print;
		string nameofcomp;
		
		//! Component Hooks
		virtual bool configureHook();
		virtual bool startHook();
		virtual void updateHook();
		
		Compare(const string& name);
		~Compare();
	};
}
#endif
