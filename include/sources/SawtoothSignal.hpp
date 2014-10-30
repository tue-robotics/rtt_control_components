/** SawtoothSignal.hpp
 *
 * @class SawtoothSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SAWTOOTHSIGNAL_HPP
#define SAWTOOTHSIGNAL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a sawtooth signal.
   *
   * The component has no inputs and only one output port of a double
   * type.
   *
   * @param * slope [-] - sets the amplitude of
   *          the generated signal.
   *        * period [sec] - period for which the
   *          generated signal is repeated.
   *        * vector_size [0] - size of output vectors
   */

  class SawtoothSignal
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<doubles> outport;

		/* Declaring global variables */
		std::vector<unsigned int> k;
		double Ts;		
		
		/* Declaring variables set by properties */
		doubles slope;
		doubles period;
		uint vector_size;
		
    public:

		SawtoothSignal(const string& name);
		~SawtoothSignal();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
