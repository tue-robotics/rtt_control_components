#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "ThreadingTestContr.hpp"

using namespace std;
using namespace RTT;
using namespace THREADINGTESTCONTR;

ThreadingTestContr::ThreadingTestContr(const string& name) : TaskContext(name, PreOperational)
{
	ThreadingTestRead = NULL;
}
ThreadingTestContr::~ThreadingTestContr(){}

bool ThreadingTestContr::configureHook()
{
	// Adding ports
	addPort( "out", outport);
	addEventPort( "in", inport);
	
	return true;
}

bool ThreadingTestContr::startHook()
{
	if ( !outport.connected() ) {
		log(Warning)<< "ThreadingTestContr::startHook: Outputport not connected!"<<endlog();
	}
	if ( !inport.connected() ) {
		log(Warning)<< "ThreadingTestContr::startHook: Inputport not connected!"<<endlog();
	}
	
	// Set Taskcontext pointer to ThreadingTestRead peer
	if ( hasPeer( "READ" ) ) {
		ThreadingTestRead 		= getPeer( "READ" );
	}
	else {
        log(Error)<< "ThreadingTestContr::startHook: Could not start component. Could not access peer: READ" << endlog();
		return false;
	}
	
	// Fetch Operation
	ResetEnc 	= ThreadingTestRead->getOperation(		"ResetEnc");
	
	// Set execution engine (this->engine():
	ResetEnc.setCaller(	ThreadingTestRead->engine());
	
	// Check if ResetEnc is ready
	if ( !ResetEnc.ready() ) {
        log(Error) << "ThreadingTestContr::startHook: Could not start component: Could not find : ThreadingTestRead.ResetEnc Operation!"<<endlog();
        return false;
    }

	return true;
}

void ThreadingTestContr::updateHook()
{
	// init
	doubles input(1,0.0);
	doubles output(1,0.0);
	
	// read inports
	inport.read(input);
	
	// write warning
	log(Warning)<< "ThreadingTestContr::updateHook: CONTR: " << input[0] << "!"<<endlog();
	
	// add 0.01 to output
	output[0] = input[0] + 0.01;

	// write output
	outport.write(output);
	
	if (output[0]>10.0) {
		ResetEnc();
		log(Warning)<< "ThreadingTestContr::updatehook: requested reset!"<<endlog();
	}

}

ORO_CREATE_COMPONENT(THREADINGTESTCONTR::ThreadingTestContr)
