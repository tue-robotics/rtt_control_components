/** RTTPLOT.hpp
 *
 * @class ConstantBool
 *
 * \author Janno Lunenburg
 * \date December, 2015
 * \version 1.0
 *
 */

#ifndef RTTPLOT_HPP
#define RTTPLOT_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace SINKS
{
  
  /**
   * @brief A Component to visualize RTT signals in realtime
   */
   
  class RTTPlot
  : public RTT::TaskContext
    {
    private:

		double Ts;
		
    public:

        RTTPlot(const string& name);
        ~RTTPlot();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
