/** Equals.hpp
 *
 * @class Equals
 *
 * \author Tim Clephas
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef EQUALS_HPP
#define EQUALS_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace MATH
{
  /**
   * @brief A Component that sets value 1 on output for positive input,
   *        value -1 for negative input, and value 0 for 0 input
   *
   * The component has one input port that should receive scalar.
   * The input port is an eventport which will trigger the component.
   *
   * @param No parameters
   */

  typedef vector<double> doubles;

  class Equals
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring global variables */
		doubles values;

	
    public:

		Equals(const string& name);
		~Equals();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
