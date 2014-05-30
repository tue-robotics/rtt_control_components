/*! 
 * \author Max Baeten
 * \date May, 2014
 * \version 1.0 
 */
 #include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>

using namespace std;
using namespace RTT;

#define PI 3.1415926535897932384626433

namespace ARM
{
	typedef std::vector<double> doubles;
	
	/*! \class GravityTorques
	 *  \brief Defines Orocos component for computation of torque in the
	 *         joints as a result of the gravitaty
	 * 
	 * Using the Recursive Newton-Euler procedure the torques in the 
	 * joints resulting from the gravity are computed. The 
	 * Denavit-Hartenberg convention is used to provide a robot model. 
	 * Note that at only revolute joints are supported by the algorithm.
	 */
	
	class GravityTorques : 

	  public RTT::TaskContext
	  {
	  private:
	  
	  InputPort<doubles> jointAnglesPort;
	  OutputPort<doubles> gravCompPort;
	  
	  int rv;
	  size_t nrJoints;
	  size_t nrCompensatedJoints;
	  
	  Eigen::MatrixXd a;
	  Eigen::MatrixXd d;
	  Eigen::MatrixXd alpha;
	  Eigen::MatrixXd coglist;
	  Eigen::MatrixXd mlist;
	  Eigen::MatrixXd Istore;
	  Eigen::MatrixXd grav;
	  Eigen::MatrixXd q;
	  Eigen::MatrixXd gravComp;

	public:

	  GravityTorques(const std::string& name);
	  ~GravityTorques();
	  
	  bool configureHook();
	  bool startHook();
	  void updateHook();
	  
	  Eigen::Matrix3d ComputeRotationMatrix(double d,double alpha,double q);
	  Eigen::Matrix3d eul2rot(double phi, double theta, double psi);
	  Eigen::MatrixXd rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd qd,Eigen::MatrixXd qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext);
	  Eigen::MatrixXd ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav);
	  
	};
	
};

