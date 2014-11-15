#ifndef RGB_CONTROLLER_HPP
#define RGB_CONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "std_msgs/ColorRGBA.h"


using namespace std;
using namespace RTT;

namespace LIGHTING {

typedef std::vector<double> doubles;

class rgb_controller : public RTT::TaskContext {
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
			rgb_controller(const std::string& name);
			~rgb_controller(){};

			bool configureHook();
			bool startHook();		  
			void updateHook();
			void stopHook(){};
		
		private:
		
			
	};
}
#endif
