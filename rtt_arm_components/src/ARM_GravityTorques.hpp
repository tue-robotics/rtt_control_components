/*!
* \author Max Baeten
* \date May, 2014
* \version 1.0 
*/

#include <vector>
#include <math.h>
#include <rtt/os/TimeService.hpp>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Eigen>

using namespace std;
using namespace RTT;
using namespace KDL;
using namespace Eigen;

#define PI 3.1415926535897932384626433

namespace ARM
{
	typedef std::vector<double> doubles;
    typedef std::vector<double> ints;
	
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
	
	//Properties
	uint nrJoints;
	doubles DH_a;
	doubles DH_d;
	doubles DH_alpha;
	doubles DH_theta;
	doubles masses;
	doubles COGx;
	doubles COGy;
	doubles COGz;
	doubles jointAngles;
	
	KDL::Chain RobotArmChain;
    KDL::ChainJntToJacSolver* jacobian_solver;
	
	//variables
	ints mass_indexes;
	uint nrMasses;
	Eigen::VectorXd GravityWrench;
	Eigen::MatrixXd GravityWrenches;
	
	public:
	
	GravityTorques(const std::string& name);
	~GravityTorques();
	
	bool configureHook();
	bool startHook();
	void updateHook();
	
	KDL::Jacobian ComputeJacobian(KDL::JntArray q_current_, int chain_tip_index);
	doubles ComputeGravityTorques(KDL::JntArray q_current_);

	};
};
