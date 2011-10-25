/** ConstantSignal.hpp
 *
 * @class ConstantSignal
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef CONSTANTSIGNAL_HPP
#define CONSTANTSIGNAL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

namespace SOURCES
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that generates constant signal
   *
   * The component has no input ports and N output ports with vector of
   * doubles
   *
   * @param * value [-] - constant value of the generated signal
   *        * vector_size [0] - size of output vectors
   */
   
  class ConstantSignal
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<doubles> outport;

		double Ts;

		/* Declaring variables set by properties */
		doubles value;
		uint vector_size;
		
    public:

		ConstantSignal(const string& name);
		~ConstantSignal();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
