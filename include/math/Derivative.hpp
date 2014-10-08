/** Derivative.hpp
 *
 * @class Derivative
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef DERIVATIVE_HPP
#define DERIVATIVE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace std;
using namespace RTT;

namespace MATH
{
	typedef vector<double> doubles;
  
  /**
   * @brief A Component that does the derivation of the input signal
   *
   * The component has one input port that should receive vector.
   *
   * @param * kv [0.0] - proportional gain
   *        * Ts [0.0] - sampling time
   *        * vector_size [0] - size of the input vector
   */
   
  class Derivative
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
		long double dt;
		long double old_time;
		// Numerator and denominator of the filter
		std::vector<double> a;
		std::vector<double> b;
		
		/* Declaring variables set by properties */
		// Filter parameters
		uint vector_size;
		double Ts;
		
		/* Declaring private functions*/
		void determineDt();
		
    public:

		Derivative(const string& name);
		~Derivative();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
