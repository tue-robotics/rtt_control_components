/** PulseSignal.hpp
 *
 * @class PulseSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef PULSESIGNAL_HPP
#define PULSESIGNAL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a pulse signal.
   *
   * The component has no inputs and only one output port works with
   * vector of doubles.
   *
   * @param * amplitude [-] - sets the amplitude of
   *          the generated signal.
   *        * period [s] - period for which the
   *          generated signal is repeated.
   *        * pulse_width [s] - width of the 
   *          generated signal where it has the set amplitude
   *          (otherwise zero).
   *        * phase_delay [s] - time for which the signal
   *          is shifted.
   *        * vector_size [0] - size of output vectors
   */

  class PulseSignal
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<doubles> outport;

		/* Declaring global variables */
		std::vector<unsigned int> k;
		double Ts;		
		
		/* Declaring variables set by properties */
		doubles amplitude;
		doubles period;
		doubles pulse_width;
		doubles phase_delay;
		uint vector_size;
		
    public:

		PulseSignal(const string& name);
		~PulseSignal();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
