/** ConstantBool.cpp
*
* @class ConstantBool
*
* \author Janno Lunenburg
* \date September, 2013
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ConstantBool.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

ConstantBool::ConstantBool(const string& name) : 
	TaskContext(name, PreOperational)
{
  addProperty( "value", value ).doc("A bool, that determines  whether the output is true or false");
}

ConstantBool::~ConstantBool(){}

bool ConstantBool::configureHook()
{
  
  // Adding ports
  addPort( "out", outport ).doc("Bool");

  Ts = getPeriod();
  
  return true;
}

bool ConstantBool::startHook()
{
  
  if ( !outport.connected() ) {
    log(Warning)<<"Output port not connected!"<<endlog();
  }

  if (Ts <= 0.0) {
    log(Error)<<"Period of the component not valid!"<<endlog();
    return false;
  }

  return true;
}

void ConstantBool::updateHook()
{
  // Write the outputs
  soem_beckhoff_drivers::DigitalMsg valuemsg;
  valuemsg.values.resize(4);
  valuemsg.values[0] = false;
  valuemsg.values[1] = true;
  valuemsg.values[2] = false;
  valuemsg.values[3] = true;
  outport.write( valuemsg );
}

ORO_CREATE_COMPONENT(SOURCES::ConstantBool)
