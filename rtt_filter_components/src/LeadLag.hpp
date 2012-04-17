/** LeadLag.hpp
 *
 * @class LeadLag
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef LEADLAG_HPP
#define LEADLAG_HPP

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
   * @brief A Component that acts as a lead-lag filters for multiple 
   *        inputs
   *
   * The component has one input port that should receive vector of 
   * doubles.
   *
   * @param * fz[] [Hz] - array of zero frequencies of the filter
   *        * fp[] [Hz] - array of pole frequencies of the filter
   *        * vector_size - size of the array
   *        * Ts - sampling time
   */
   
  class LeadLag
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
		doubles a0;
		doubles a1;
		doubles b0;
		doubles b1;
		
		/* Declaring variables set by properties */
		// Filter parameters
		doubles fz;
		doubles fp;
		uint vector_size;
		double Ts;
		
    public:

		LeadLag(const string& name);
		~LeadLag();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
