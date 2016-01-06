#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "ThreadingTestBase.hpp"

using namespace std;
using namespace RTT;
using namespace THREADINGTESTBASE;

ThreadingTestBase::ThreadingTestBase(const string& name) : TaskContext(name, PreOperational)
{
	addProperty( "compname",        compname        ).doc("Name of comp");

}
ThreadingTestBase::~ThreadingTestBase(){}

bool ThreadingTestBase::configureHook()
{
	// Adding ports
	addPort( "out", outport);
	addEventPort( "in", inport);
	
	return true;
}

bool ThreadingTestBase::startHook()
{

	if ( !outport.connected() ) {
		log(Warning)<< compname << "::startHook: Outputport not connected!"<<endlog();
	}
	if ( !inport.connected() ) {
		log(Warning)<< compname << "::startHook: Inputport not connected!"<<endlog();
	}

	return true;
}

void ThreadingTestBase::updateHook()
{
	// init
	doubles input(1,0.0);
	doubles output(1,0.0);
	
	// read inports
	inport.read(input);
	
	// write warning
	log(Warning)<< compname << "::updateHook: Received: " << input[0] << " !"<<endlog();

	// add 0.01 to output
	output[0] = input[0] + 0.01;

	// write output
	outport.write(output);
}

ORO_CREATE_COMPONENT(THREADINGTESTBASE::ThreadingTestBase)
