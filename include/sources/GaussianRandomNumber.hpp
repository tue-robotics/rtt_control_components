/** GaussianRandomNumber.hpp
 *
 * @class GaussianRandomNumber
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef GAUSSIANRANDOMNUMBER_HPP
#define GAUSSIANRANDOMNUMBER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <math.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates a gaussian random number
   *
   * The component has no inputs and only one output port of a double
   * type.
   *
   * @param * mean [-] - sets the mean value of the generated signals
   *        * variance [-] - sets the variance of the generated signals
   *        * vector_size [0] - size of output vectors
   */
   
  class GaussianRandomNumber
  : public RTT::TaskContext
    {
    private:

		/*** Declaring output port ***/
		OutputPort<doubles> outport;

		/*** Declaring global variables ***/
		double Ts;
		
		/*** Declaring variables set by properties ***/
		doubles mean;
		doubles variance;
		uint vector_size;
		
		double randomnumgen(double, double);
		double box_muller(double m, double v);	
		
    public:

		GaussianRandomNumber(const string& name);
		~GaussianRandomNumber();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
