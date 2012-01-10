/** ReporterTrigger.hpp
 *
 * @class ReporterTrigger
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef ReporterTrigger_HPP
#define ReporterTrigger_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace RTT;

namespace CUSTOM
{
  // Define a new type for easy coding:
  typedef vector<double> doubles;
  
  /**
   * @brief A Component that calculates ReporterTriggerolute value of the 
   *        input signals
   *
   * The component has input port as stated in property number_of_inputs,
   * that should receive vector of doubles.
   * Input ports are eventports which will trigger the component.
   * 
   * @param * N - number of inputs
   *        * vector_size - size of input vectors
   */
   
  class ReporterTrigger
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;	
	
    public:

		ReporterTrigger(const string& name);
		~ReporterTrigger();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
