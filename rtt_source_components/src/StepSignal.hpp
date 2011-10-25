/** StepSignal.hpp
 *
 * @class StepSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef STEPSIGNAL_HPP
#define STEPSIGNAL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace RTT;

namespace SOURCES
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a step signal.
   *
   * The component has no inputs and only one output port that works with
   * vector of doubles.
   *
   * @param * step_time [s] - sets the time point at which the
   *          signal changes its value from initial to final
   *        * init_value [-] - initial value of the signal
   *        * final_value [-] - final value of the signal
   *        * vector_size [0] - size of output vectors
   */
   
  class StepSignal
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<doubles> outport;

		/* Declaring global variables */
		unsigned int k;
		double Ts;		
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles step_time;
		doubles init_value;
		doubles final_value;
		uint vector_size;
		
    public:

		StepSignal(const string& name);
		~StepSignal();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
