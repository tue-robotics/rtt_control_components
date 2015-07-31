/** Switch.hpp
 *
 * @class Switch
 *
 * \author Max Baeten
 * \date June, 2013
 * \version 1.0     Note that this component is not tested, and never used at this point 
 *
 */

#ifndef Switch_HPP
#define Switch_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <string>
#include <iostream>
#include <sstream>

#define maxN 10 //Maximum number of ports that can be created

using namespace std;
using namespace RTT;
namespace SIGNALROUTING
{
	typedef vector<double> doubles;

    /**
   * @brief A Component that Switches the input signals, i.e. 
   *        switches two input signals to one port. Usefull for
   * 		switching of Homing component and ReadReference component
   * 
   * 		This component switches from port in_default
   * 		to port in_switched, 
   *		when switch_param becomes true
   * 
   */

  class Switch
  : public RTT::TaskContext
    {
    private:

		// Declaring and output ports
        InputPort<bool> inport_switchParam; //to Do make service call not port
		InputPort<doubles> inport_default;
		InputPort<doubles> inport_switched;
		OutputPort<doubles> outport;
		
		bool switch_param;
        uint N;
		doubles input_default;
        doubles input_switched;
		doubles output;
		
    public:

		Switch(const string& name);
		~Switch();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
