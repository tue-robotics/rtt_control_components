#include "ROSinterface.hpp"

using namespace std;
using namespace RTT;
using namespace ROSINTERFACE;

ROSinterface::ROSinterface(const string& name) :
    TaskContext(name, PreOperational)
{

}

ROSinterface::~ROSinterface(){}

bool ROSinterface::configureHook()
{
	
}

bool ROSinterface::startHook()
{
	
}

void ROSinterface::updateHook()
{
	
}

ORO_CREATE_COMPONENT(ROSINTERFACE::ROSinterface)
