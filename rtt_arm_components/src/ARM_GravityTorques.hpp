/*!
* \author Max Baeten
* \date May, 2014
* \version 1.0 
*/

#include <rtt/os/TimeService.hpp>
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
      OutputPort<doubles> gravityTorquesPort;
	  
	  size_t nrJoints;
	  
      Eigen::MatrixXd DH_a;
      Eigen::MatrixXd DH_d;
      Eigen::MatrixXd DH_alpha;
      Eigen::MatrixXd DH_theta;
	  
      Eigen::MatrixXd m;
      Eigen::MatrixXd COG;

	public:

	  GravityTorques(const std::string& name);
	  ~GravityTorques();
	  
	  bool configureHook();
	  bool startHook();
	  void updateHook();

      Eigen::MatrixXd ComputeJacobian();
      doubles ComputeGravityTorques();
	  
	};
	
};

