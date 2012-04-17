/** WeakIntegrator.hpp
 *
 * @class WeakIntegrator
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef WEAKINTEGRATOR_HPP
#define WEAKINTEGRATOR_HPP

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
   * @brief A Component that acts as a weak-integrator filter
   *
   * The component has one input port that should receive scalar.
   *
   * @param * fz [Hz] - zero frequency of the filter
   *        * vector_size [0] - size of input vector
   *        * Ts [0.0 sec] - sampling time
   */
   
  class WeakIntegrator
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
		doubles fz;
		uint vector_size;
		double Ts;		
		
    public:

		WeakIntegrator(const string& name);
		~WeakIntegrator();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
