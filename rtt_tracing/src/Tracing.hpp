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

using namespace std;
using namespace RTT;

namespace Signal
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
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
		InputPort<doubles> inport;

		int buffersize;
		doubles buffer;
		vector<doubles> buffers;
		int counter;
		int columns;
		int rows;
	
    public:

		Tracing(const string& name);
		~Tracing();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();

    };
}
#endif
