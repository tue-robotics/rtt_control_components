/** RTTPlot.cpp
*
* @class RTTPlot
*
* \author Janno Lunenburg
* \date December, 2015
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "sinks/RTTPlot.hpp"

using namespace std;
using namespace RTT;
using namespace SINKS;

RTTPlot::RTTPlot(const string& name) :
	TaskContext(name, PreOperational)
{

}

RTTPlot::~RTTPlot(){}

bool RTTPlot::configureHook()
{
  
  Ts = getPeriod();
  
  return true;
}

bool RTTPlot::startHook()
{
  
  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }

  return true;
}

void RTTPlot::updateHook()
{

}

ORO_CREATE_COMPONENT(SINKS::RTTPlot)
