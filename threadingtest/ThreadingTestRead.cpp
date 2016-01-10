#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "ThreadingTestRead.hpp"

using namespace std;
using namespace RTT;
using namespace THREADINGTESTREAD;

ThreadingTestRead::ThreadingTestRead(const string& name) : TaskContext(name, PreOperational)
{
	addOperation( "ResetEnc", &ThreadingTestRead::ResetEnc, this, OwnThread )
		.doc("Zero an encoder");
}
ThreadingTestRead::~ThreadingTestRead(){}

bool ThreadingTestRead::configureHook()
{
	// Adding ports
	addPort( "out", outport);
	addEventPort( "in", inport);
	
	return true;
}

bool ThreadingTestRead::startHook()
{
	if ( !outport.connected() ) {
		log(Warning)<< "ThreadingTestRead::startHook: Outputport not connected!"<<endlog();
	}
	if ( !inport.connected() ) {
		log(Warning)<< "ThreadingTestRead::startHook: Inputport not connected!"<<endlog();
	}
	
	reset_value = 0.0;
	input.resize(1);

	return true;
}

void ThreadingTestRead::updateHook()
{
	// init
	input[0] = 0.0;
	doubles output(1,0.0);
	
	// read inports
	inport.read(input);
	
	// write warning
	log(Warning)<< "ThreadingTestRead::updateHook: READ : " << input[0] << "!"<<endlog();

	// add 0.01 to output
	output[0] = input[0] + 0.01 - reset_value;

	// write output
	outport.write(output);
}

void ThreadingTestRead::ResetEnc()
{
	reset_value = input[0];
	log(Warning)<< "ThreadingTestRead::ResetEnc: Zeroing encoder at: " << reset_value << "!"<<endlog();
}


ORO_CREATE_COMPONENT(THREADINGTESTREAD::ThreadingTestRead)
