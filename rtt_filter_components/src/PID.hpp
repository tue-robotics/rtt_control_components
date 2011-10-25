/** PID.hpp
 *
 * @class PID
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef PID_HPP
#define PID_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

namespace FILTERS
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that acts as a PID filter with anti-windup
   *
   * The component has one input port that should receive array.
   *
   * @param * kp [-] - proportional gain
   *        * kv [-] - derivative gain
   *        * ki [-] - integral gain
   *        * kaw [-] - anti-windup gain
   *        * init [-] - initial value of the integrator
   *        * limit [-] - output limit (saturation value)
   *        * Ts [0.0 sec] - sampling time
   *        * vector_size [0] - size of input vector
   */
   
  class PID
  : public RTT::TaskContext
    {
    private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		// Variables for history storage
		doubles previous_output;
		doubles previous_input;
		doubles second_previous_output;
		doubles second_previous_input;
		// Parameters for Tustin prewarping discretization
		long double old_time;
		doubles windup;
		doubles anti_windup;
		
		// Numerator and denominator of the filter
		doubles a[3];
		doubles b[3];
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles kp;
		doubles kv;
		doubles ki;
		doubles kaw;
		doubles init;
		doubles limit;
		double Ts;
		uint vector_size;
		
		double determineDt();
		
    public:

		PID(const string& name);
		~PID();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
