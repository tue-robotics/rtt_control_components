/** Deadzone.hpp
 *
 * @class Deadzone
 *
 * \author Janno Lunenburg
 * \date Februari, 2012
 * \version 1.0
 *
 */
 
#ifndef DEADZONE_HPP
#define DEADZONE_HPP

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
   * @brief Output zero for inputs within the deadzone. Offsets the input signals by either the Start or End value when outside of the deadzone
   *
   * The component has one input port that should receive vector of doubles.
   * Output signals are input signals with offset specified by deadzone
   *
   * @param * start_deadzone [-] - start of the deadzone
   *        * end_deadzone [-] - end of the deadzone
   *        * vector_size [0] - size of the input vector
   */
   
  class Deadzone
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		InputPort<doubles> inport;
		OutputPort<doubles> outport;

		/* Declaring variables set by properties */
		doubles start_deadzone;
		doubles end_deadzone;
		uint vector_size;
		
    public:

		Deadzone(const string& name);
		~Deadzone();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif

