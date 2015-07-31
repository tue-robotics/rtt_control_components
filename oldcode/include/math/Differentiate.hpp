/** Differentiate.hpp
 *
 * @class Differentiate
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef DIFFERENTIATE_HPP
#define DIFFERENTIATE_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace std;
using namespace RTT;

namespace MATH
{
	typedef vector<double> doubles;
	
  /**
   * @brief A Component that acts as a differentiator.
   *
   * It used the operator system time to calculate the derivative
   * of the input vector.
   * 
   * The component has one input port that should receive vector.
   *
   * Optionally you can use a discrete time derivative and a time
   * check if the time between two consecutive update hook function
   * calles is within bounds.
   *
   * @param * vector_size [0] - size of the input vector
   * 		* sampling_time - optional, time used for discrete 
   *		  calculation and time check.
   *		* use_time_check - use time check for the update hook
   *		* allowed_variation_Ts - +/- variation in time
   *		* use_discrete - use discrete time derivative
   */
   
  class Differentiate
  : public RTT::TaskContext
    {
    private:

		/* Declaring input and output ports*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		// Variables for history storage
		doubles previous_input;
		doubles previous_output;
		double dt;
		long double old_time;
		
		/* Declaring variables set by properties */
		uint vector_size;
		double Ts;
		double epsTs;
		bool time_check;
		bool discrete;
		
		void determineDt();
		
    public:

		Differentiate(const string& name);
		~Differentiate();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
