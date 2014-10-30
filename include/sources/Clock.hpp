/** Clock.hpp
 *
 * @class Clock
 *
 * \author Boris Mrkajic
 * \date March, 2011
 * \version 1.0
 *
 */

#ifndef Clock_HPP
#define Clock_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
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
   
  class Clock
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<doubles> outport;

		double start;

		/* Declaring variables set by properties */
		uint vector_size;
		
    public:

		Clock(const string& name);
		~Clock();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
