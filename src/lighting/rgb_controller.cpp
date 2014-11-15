#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "rgb_controller.hpp"
#include <rtt/os/TimeService.hpp>

using namespace std;
using namespace RTT;
using namespace LIGHTING;

rgb_controller::rgb_controller(const std::string& name) :
	TaskContext(name, PreOperational) {
	addPort("red", red_port);
	addPort("green", green_port);
	addPort("blue", blue_port);
	addPort("rgbtopic", rgbPort);
}

bool rgb_controller::configureHook() {
	return true;
}

bool rgb_controller::startHook() {
	counter = 0;
	
	rgbred = 0;
	rgbgreen = 0;
	rgbblue = 255;
	
	return true;
}

void rgb_controller::updateHook() {

	if ( counter == 0 )
	{
		if (rgbPort.read(rgbmsg) == NewData){
			rgbred = uint(rgbmsg.r * 255);
			rgbgreen = uint(rgbmsg.g * 255);
			rgbblue = uint(rgbmsg.b * 255);
		}
		
		red = 0.5+rgbred/12.75;
		green = 0.5+rgbgreen/12.75;
		blue = 0.5+rgbblue/12.75;
	}
	
	bool boolred = red > counter;
	bool boolgreen = green > counter;
	bool boolblue = blue > counter;

	red_port.write(boolred);
	green_port.write(boolgreen);
	blue_port.write(boolblue);
	
		
	counter++;
	if (counter > 19)
		counter = 0;

}

ORO_CREATE_COMPONENT(LIGHTING::rgb_controller)
