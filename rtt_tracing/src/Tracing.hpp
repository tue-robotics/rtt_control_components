/** Tracing.hpp
 *
 * @class Tracing
 *
 * \author Tim Clephas
 * \date Summer, 2012
 * \version 1.0
 *
 */

#ifndef Tracing_HPP
#define Tracing_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include "boost/multi_array.hpp"
#include <cassert>

#define maxN 10 //Maximum number of ports that can be created. Still a workaround.


using namespace std;
using namespace RTT;

namespace Signal
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  typedef vector<uint> uints;

  template <class T>
  inline string to_string (const T& t){
    stringstream ss;
    ss << t;
    return ss.str();
  };
  
  /**
   * @brief A Component that calculates Tracingolute value of the 
   *        input signals
   *
   * The component has input port as stated in property number_of_inputs,
   * that should receive vector of doubles.
   * Input ports are eventports which will trigger the component.
   * 
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */
   
  class Tracing
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
		bool crash;

		doubles buffer;
		vector<doubles> buffers;
		int counter;
		uint columns;
		uint rows;
	
    public:

		Tracing(const string& name);
		~Tracing();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void fatal();

    };
}
#endif
