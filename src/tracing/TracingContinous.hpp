/** TracingContinous.hpp
 *
 * @class TracingContinous
 *
 * \author Max Baeten
 * \date Feb, 2016
 * \version 1.0
 *
 */

#ifndef TracingContinous_HPP
#define TracingContinous_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include "boost/multi_array.hpp"
#include <cassert>

#define MAX_BODYPARTS 6
#define MAX_PORTS 5
#define MAX_JOINTS 10

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

namespace Signal
{
	typedef vector<double> doubles;
	typedef vector<uint> uints;
	typedef vector<string> strings;
	
	/**
	* @brief A Component that traces all relevant signals in a robot,
	* at all times. When a error occurs it will send a log file via email.
	* 
	* The component will make use of the bodypart structure, therefore it should be
	* started in soem.ops and then each bodypart should be added inside the 
	* bodypart ops files.
	* 
	* @param * 
	* 
	* Function to add a boypart to the tracing continous 
	* 
	*/

	class TracingContinous
	: public RTT::TaskContext
	{
		private:
		
		/* Declaring and output port*/
		InputPort<doubles> dataInports[MAX_BODYPARTS][MAX_JOINTS];
		InputPort<bool> errorInports[MAX_BODYPARTS];

		// Properties
		string filename;
		uint buffersize;
		uint sendErrorLog_delay;
		double Ts;

		// buffer
		bool error;
		uint error_bpid;
		uint sendErrorLog_delaycntr;
		bool buffer_full;
		uint n_totalports;
		uint n_totaltraces;
		uint buffer_nrports[MAX_BODYPARTS];
		uint buffer_nrjoints[MAX_BODYPARTS];
		uint n_cyclicbuffer;
		strings buffer_names;
		bool buffer_status[MAX_BODYPARTS];
		vector<doubles> input[MAX_BODYPARTS]; // input = vector of bodyparts, vector of ports, vector of joints // stores last sample
		vector< vector< doubles > > buffer[MAX_BODYPARTS]; // buffer = vector of bodyparts, vector of ports, vector of joints, vector of ndatapoints
		
		public:

		TracingContinous(const string& name);
		~TracingContinous();

		bool configureHook();
		bool startHook();
		void updateHook();
		virtual void AddBodypart(string PARTNAME, uint BPID, uint NRPORTS, uint NRJOINTS, strings PORTNAMES);
		virtual void sendErrorLog(int BPID, uint N_CYCLICBUFFER);
		virtual void stopHook(int BPID, uint N_CYCLICBUFFER);
	};
}
#endif
