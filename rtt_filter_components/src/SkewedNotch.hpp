/** SkewedNotch.hpp
 *
 * @class SkewedNotch
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef SKEWEDNOTCH_HPP
#define SKEWEDNOTCH_HPP

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
   * @brief A Component that acts as a skewed-notch filter
   *
   * The component has one input port that should receive vector of doubles.
   *
   * @param * fz [Hz] - zero frequency of the filter
   *        * dz [-] - zero damping
   *        * fp [Hz] - pole frequency of the filter
   *        * dp [-] - pole damping
   *        * vector_size [0] - size of input vector
   *        * Ts [0.0 sec] - sampling time
   */
   
  class SkewedNotch
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
		doubles a[3];
		doubles b[3];
		
		long double old_time;
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles fz;
		doubles dz;
		doubles fp;
		doubles dp;
		uint vector_size;
		double Ts;		
				
		double determineDt();
		
    public:

		SkewedNotch(const string& name);
		~SkewedNotch();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
