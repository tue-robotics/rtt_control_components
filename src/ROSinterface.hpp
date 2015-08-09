#ifndef ROSINTERFACE_HPP_
#define ROSINTERFACE_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#define MAX_PORTS 10 /* maximum number of ports */

using namespace std;
using namespace RTT;

template <class T>
inline string to_string (const T& t) { stringstream ss; ss << t; return ss.str(); }

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
		
		// Doubles2ROS
		InputPort<doubles> inports_D[MAX_PORTS];
		OutputPort<std_msgs::Float32MultiArray> outports_D[MAX_PORTS];
		uint n_doubles_2ROS;
		doubles input_D[MAX_PORTS];
		std_msgs::Float32MultiArray output_D[MAX_PORTS];
		
		// Digital2ROS
		InputPort<bool> inports_B[MAX_PORTS];
		OutputPort<std_msgs::Bool> outports_B[MAX_PORTS];
		uint n_bool_2ROS;
		bool input_B[MAX_PORTS];
		std_msgs::Bool output_B[MAX_PORTS];
		
		// Jointstate2ROS
		InputPort<doubles> inports_J[MAX_PORTS];
		OutputPort<sensor_msgs::JointState> outports_J;		
		uint n_jointstate_2ROS;
		doubles input_J[MAX_PORTS];
		std_msgs::Float32MultiArray output_J[MAX_PORTS];
		

        //! Component Hook functions
        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();

		virtual void AddDoubles2ROS(uint SIZE);
		virtual void AddBool2ROS();
		virtual void AddJointstate2ROS(uint SIZE);

        ROSinterface(const string& name);
        virtual ~ROSinterface();    

		protected:


      };
}

#endif
