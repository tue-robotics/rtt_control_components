/** UniformRandomNumber.hpp
 *
 * @class UniformRandomNumber
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef UNIFORMRANDOMNUMBER_HPP
#define UNIFORMRANDOMNUMBER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a uniform random number
   *
   * The component has no inputs and only one output port of a double
   * type.
   *
   * @param * maximum [-] - sets the maximum value of the generated signals
   *        * minimum [-] - sets the minimum value of the generated signals
   *        * vector_size [0] - size of output vectors
   */
   
  class UniformRandomNumber
  : public RTT::TaskContext
    {
    private:

		/*** Declaring output port ***/
		OutputPort<doubles> outport;

		/*** Declaring global variables ***/
		double Ts;
		
		/*** Declaring variables set by properties ***/
		doubles maximum;
		doubles minimum;
		uint vector_size;
		
		double randomnumgen(double, double);
		
    public:

		UniformRandomNumber(const string& name);
		~UniformRandomNumber();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
