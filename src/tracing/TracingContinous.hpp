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

#define maxN 10 //Maximum number of ports that can be created.


using namespace std;
using namespace RTT;

namespace Signal
{
  typedef vector<double> doubles;
  typedef vector<uint> uints;

  template <class T>
  inline string to_string (const T& t){
    stringstream ss;
    ss << t;
    return ss.str();
  };
  
  /**
   * @brief A Component that traces all relevant signals in a robot,
   * at all times. When a error occurs it will send a log file via email.
   * 
   * The component will make use of the bodypart structure, therefore it should be
   * started in soem.ops and then each bodypart should be added inside the 
   * bodypart ops files.
   * 
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */
   
  class TracingContinous
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inports[maxN];

		string filename;
		doubles vectorsizes_prop;
		vector<int> vectorsizes;
		uint buffersize;
		double Ts;

		doubles buffer;
		vector<doubles> buffers;
		int counter;
		uint columns;
		uint rows;
		uint Nports;
		bool printed;
	
    public:

		TracingContinous(const string& name);
		~TracingContinous();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void Send_Email(char * msgbuf);

    };
}
#endif
