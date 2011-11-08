/** SecondOrderLowPasses.hpp
 *
 * @class SecondOrderLowPasses
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SECONDORDERLOWPASSES_HPP
#define SECONDORDERLOWPASSES_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace RTT;

namespace FILTERS
{
  
  // Define a new type for easy coding:
  typedef vector<double> doubles;
	
  /**
   * @brief A Component that acts as a 2nd order low-pass filter
   *
   * The component has one input port that should receive scalar.
   *
   * @param * fp [100 Hz] - pole frequency of the filter
   *        * dp [0.1] - pole damping
   */
   
  class SecondOrderLowPasses
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

		// Numerator and denominator of the filter
		doubles a0;
		doubles a1;
		doubles a2;
		doubles b0;
		doubles b1;
		doubles b2;
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles fp;
		doubles dp;
		uint vector_size;
		double Ts;
		
    public:

		SecondOrderLowPasses(const string& name);
		~SecondOrderLowPasses();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
