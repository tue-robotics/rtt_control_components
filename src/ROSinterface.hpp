#ifndef ROSINTERFACE_HPP_
#define ROSINTERFACE_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/UInt8MultiArray.h>
#include <rosgraph_msgs/Log.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace ROSINTERFACE
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	
	/*! \class ROSinterface
	 *  \brief Handles all ROS communication
	 *
	 */

    class ROSinterface
    : public RTT::TaskContext
      {
		public:

        //! Component Hook functions
        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();

        ROSinterface(const string& name);
        virtual ~ROSinterface();    

		protected:


      };
}

#endif
