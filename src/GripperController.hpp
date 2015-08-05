#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <std_msgs/Bool.h>
#include <tue_msgs/GripperCommand.h>
#include <tue_msgs/GripperMeasurement.h>
#include <amigo_ref_interpolator/interpolator.h>

#define PI 3.141592654
#define MAX_TORQUE 150.0
#define GRIPPER_INDEX	7                       

using namespace std;
using namespace RTT;

namespace GRIPPERCONTROLLER
{
	typedef std::vector<double> doubles;
	
	/*! \class GripperControl
	 *  \brief Defines Orocos component for controlling the gripper
	 * 
	 * The GripperControl closes the gripper using a force threshold. It
	 * opens the gripper to a predefined position.
	 */
	
	class GripperController : 

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
      OutputPort<doubles> posoutport;
      OutputPort<doubles> veloutport;
      OutputPort<doubles> accoutport;
	  OutputPort<tue_msgs::GripperMeasurement> gripperMeasurementPort;
	  
	  // Properties
	  uint sensorPos;
	  double maxPos;
	  double minPos;
	  double threshold_closed;
	  double desiredPos;
      double desiredVel;
      double desiredAcc;
      double InterpolDt;
      double InterpolEps;
	  
	  // variables
  	  bool completed;
	  bool gripperHomed;
	  doubles torques;
	  doubles measPos;
      tue_msgs::GripperCommand gripperCommand;
      refgen::RefGenerator mRefGenerator;
      amigo_msgs::ref_point mRefPoint;
	  
	public:

	  GripperController(const std::string& name);	  
	  ~GripperController();
	  
	  bool configureHook();
	  bool startHook();
	  void updateHook();
	};
	
};

