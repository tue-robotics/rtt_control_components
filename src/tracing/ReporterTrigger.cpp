/** ReporterTrigger.cpp
*
* @class ReporterTrigger
*
* \author Boris Mrkajic
* \date March, 2011
* \version 1.0
*
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "ReporterTrigger.hpp"

using namespace std;
using namespace RTT;
using namespace CUSTOM;

ReporterTrigger::ReporterTrigger(const string& name) : 
	TaskContext(name, PreOperational)
{
	addEventPort( "in", inport);
	addPort( "out", outport);
}

ReporterTrigger::~ReporterTrigger(){}

bool ReporterTrigger::configureHook()
{
	Logger::In in("ReporterTrigger::Configure");	
	
      if( ! hasPeer("Reporter") )
    {
        log(Error) << "You can't trigger a component that is not your peer !" << endlog();
        return false;
    }
  return true;
}

bool ReporterTrigger::startHook()
{
	Logger::In in("ReporterTrigger::Start");	

	if ( !inport.connected() ) {
		log(Error)<<"Input port not connected!"<<endlog();
		return false;
	}
	
	if ( !outport.connected() ) {
		log(Warning)<<"Output port not connected!"<<endlog();
	}
	
	return true;
}

void ReporterTrigger::updateHook()
{
	Logger::In in("ReporterTrigger::Update");	
	
	doubles input;

	inport.read( input );
	outport.write( input );
	getPeer("Reporter")->trigger();
}

ORO_CREATE_COMPONENT(CUSTOM::ReporterTrigger)
