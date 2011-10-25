/** RampSignal.hpp
 *
 * @class RampSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef RAMPSIGNAL_HPP
#define RAMPSIGNAL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

namespace SOURCES
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a ramp signal.
   *
   * The component has no inputs and only one output port of vector of
   * double values.
   *
   * @param * slope [-] - sets the slope of the ramp.
   *        * vector_size [1] - size of output vectors.
   */
   
  class RampSignal
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
		doubles slope;
		uint vector_size;
		
    public:

		RampSignal(const string& name);
		~RampSignal();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
