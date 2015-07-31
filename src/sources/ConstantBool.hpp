/** ConstantBool.hpp
 *
 * @class ConstantBool
 *
 * \author Janno Lunenburg
 * \date September, 2013
 * \version 1.0
 *
 */

#ifndef CONSTANTBOOL_HPP
#define CONSTANTBOOL_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <soem_beckhoff_drivers/DigitalMsg.h>

using namespace std;
using namespace RTT;

namespace SOURCES
{
  
  /**
   * @brief A Component that generates constant boolean signal
   *
   * The component has no input ports and N output ports with vector of
   * doubles
   *
   * @param * value [-] - constant value of the generated signal: true or false
   */
   
  class ConstantBool
  : public RTT::TaskContext
    {
    private:

		/* Declaring and output port*/
		OutputPort<soem_beckhoff_drivers::DigitalMsg> outport;

		double Ts;

		/* Declaring variables set by properties */
		bool value;
		
    public:

		ConstantBool(const string& name);
		~ConstantBool();

		bool configureHook();
		bool startHook();
		void updateHook();

    };
}
#endif
