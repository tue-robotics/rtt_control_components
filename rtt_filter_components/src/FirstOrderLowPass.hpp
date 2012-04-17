/** FirstOrderLowPass.hpp
 *
 * @class FirstOrderLowPass
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef FIRSTORDERLOWPASS_HPP
#define FIRSTORDERLOWPASS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <vector>

using namespace std;
using namespace RTT;

namespace FILTERS
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
	
  /**
   * @brief A Component that acts as a 1st order low-pass filter
   *
   * The component has one input port that should receive scalar.
   *
   * @param * fp [Hz] - pole frequency of the filter
   *        * vector_size [0] - size of input vector
   *        * Ts [0.0 sec] - sampling time
   */
   
  class FirstOrderLowPass
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

		// Numerator and denominator of the filter
		doubles a[2];
		doubles b[2];
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles fp;
		uint vector_size;
		double Ts;
		
    public:

		FirstOrderLowPass(const string& name);
		~FirstOrderLowPass();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
