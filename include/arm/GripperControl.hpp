/**************************************************************************
 *                                                                        *
 *   S. Marinkov                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <tue_msgs/GripperCommand.h>
#include <tue_msgs/GripperMeasurement.h>

#define PI 3.141592654
#define GRIPPER_INDEX	7

using namespace std;
using namespace RTT;

namespace ARM
{
	typedef std::vector<double> doubles;
	
	/*! \class GripperControl
	 *  \brief Defines Orocos component for controlling the gripper
	 * 
	 * The GripperControl closes the gripper using a force threshold. It
	 * opens the gripper to a predefined position.
	 */
	
	class GripperControl : 

	  public RTT::TaskContext
	  {
	  private:
	  // Inports
	  InputPort<tue_msgs::GripperCommand> gripperCommandPort;
	  InputPort<doubles> torqueInPort;
	  InputPort<doubles> positionInPort;
	  InputPort<bool> resetGripperPort;
	  InputPort<bool> reNullPort;
	  
	  // Outports
	  OutputPort<doubles> gripperRefPort;
	  OutputPort<tue_msgs::GripperMeasurement> gripperMeasurementPort;
	  
	  // Properties
	  uint sensorPos;
	  double maxPos;
	  double gripperGain;
	  double threshold_closed;
	  
	  // variables
  	  bool completed;
	  bool gripperHomed;
	  doubles torques;
	  doubles measPos;
	  doubles gripperPos;
      tue_msgs::GripperCommand gripperCommand;
	  
	public:

	  GripperControl(const std::string& name);	  
	  ~GripperControl();
	  
	  bool configureHook();
	  bool startHook();
	  void updateHook();
	};
	
};

