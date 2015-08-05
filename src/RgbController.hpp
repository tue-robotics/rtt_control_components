#ifndef RGBCONTROLLER_HPP
#define RGBCONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>

#include "std_msgs/ColorRGBA.h"


using namespace std;
using namespace RTT;

namespace RGBCONTROLLER {

typedef std::vector<double> doubles;

class RgbController : public RTT::TaskContext {
		private:

			OutputPort<bool> red_port;
			OutputPort<bool> green_port;
			OutputPort<bool> blue_port;
			InputPort<std_msgs::ColorRGBA> rgbPort;
			int counter;
			
			std_msgs::ColorRGBA rgbmsg;
			
			uint rgbred;
			uint rgbgreen;
			uint rgbblue;	

			int red;
			int green;
			int blue;

		public:
			RgbController(const std::string& name);
			~RgbController(){};

			bool configureHook();
			bool startHook();		  
			void updateHook();
			void stopHook(){};
		
		private:
		
			
	};
}
#endif
