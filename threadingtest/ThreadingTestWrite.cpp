#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "ThreadingTestWrite.hpp"

using namespace std;
using namespace RTT;
using namespace THREADINGTESTWRITE;

ThreadingTestWrite::ThreadingTestWrite(const string& name) : TaskContext(name, PreOperational)
{
	
}
ThreadingTestWrite::~ThreadingTestWrite(){}

bool ThreadingTestWrite::configureHook()
{
	// Adding ports
	addPort( "out", outport);
	addEventPort( "in", inport);
	
	return true;
}

bool ThreadingTestWrite::startHook()
{

	if ( !outport.connected() ) {
		log(Warning)<< "ThreadingTestWrite::startHook: Outputport not connected!"<<endlog();
	}
	if ( !inport.connected() ) {
		log(Warning)<< "ThreadingTestWrite::startHook: Inputport not connected!"<<endlog();
	}

	return true;
}

void ThreadingTestWrite::updateHook()
{
	// init
	doubles input(1,0.0);
	doubles output(1,0.0);
	
	// read inports
	inport.read(input);
	
	// write warning
	log(Warning)<< "ThreadingTestWrite::updateHook: WRITE: " << input[0] << "! \n"<<endlog();

	// add 0.01 to output
	output[0] = input[0] + 0.01;

	// write output
	outport.write(output);
}

ORO_CREATE_COMPONENT(THREADINGTESTWRITE::ThreadingTestWrite)
