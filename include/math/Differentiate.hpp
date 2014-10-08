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
   * @brief A Component that acts as a differentiator
   *
   * The component has one input port that should receive vector.
   *
   * @param * vector_size [0] - size of the input vector
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

		double dt;
		long double old_time;
		
		/* Declaring variables set by properties */
		uint vector_size;
		
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
