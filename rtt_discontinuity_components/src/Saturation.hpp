/** Saturation.hpp
 *
 * @class Saturation
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */
 
#ifndef SATURATION_HPP
#define SATURATION_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

using namespace RTT;

namespace DISCONTINUITIES
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that saturates the input signal
   *
   * The component has one input port that should receive vector of doubles.
   * The output is saturated between upper and lower limit.
   *
   * @param * upper_limit [-] - upper saturation limits
   *        * lower_limit [-] - lower saturation limits
   *        * vector_size [0] - size of the input vector
   */
   
  class Saturation
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring variables set by properties */
		doubles upper_limit;
		doubles lower_limit;
		uint vector_size;
		
    public:

		Saturation(const string& name);
		~Saturation();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
