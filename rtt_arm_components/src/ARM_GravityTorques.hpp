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
    typedef std::vector<int> ints;
    typedef std::vector<string> strings;
	
	/*! \class GravityTorques
     *  \brief Defines Orocos component for computation of joint torques
     *  as a result of gravity. Note that only positive revolute joints are
     *  supported however support of prismatic joints or negative revolute
     *  joints could be implemented. Also for each link either a rotation
     *  in theta or in alpha can be made, not both.
	 * 
     * The Denavit-Hartenberg convention is used to provide a robot model.
     * To indicate the joint directions, rot_Z, and rot_X are used.
     *
     * Every link that has a mass, should be provided with coordinates
     * of the location of the Centor of gravity using COGx, COGy, and COGz.
     *
     * These COG's are then considered point masses and for each of them a
     * gravitywrench is calculated (one column in GravityWrenches)
     *
     * A partial jacobian matrix is then calculated for each COG to the
     * base frame.
     *
     * Using partial_jacobian_.transpose() * GravityWrenches.col(i). the
     * gravity joint torques of the ith COG is calculated summing to the
     * total output gravityTorques.
	 */
	
	class GravityTorques : 
	
	public RTT::TaskContext
	{
	private:
	
	InputPort<doubles> jointAnglesPort;
	OutputPort<doubles> gravityTorquesPort;
	
	//Properties
	uint nrJoints;
    strings joint_type;
    strings joint_axis;
    strings rotation_axis;
    doubles GravityVector;
    doubles rotation_angle;
    doubles translation_X;
    doubles translation_Y;
    doubles translation_Z;
	doubles masses;
	doubles COGx;
	doubles COGy;
	doubles COGz;

    // temp
    bool printed;

    // input
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
