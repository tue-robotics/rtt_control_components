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

#define MAXJOINTS 10

namespace ARM
{
	typedef std::vector<double> doubles;
    typedef std::vector<uint> ints;
    typedef std::vector<string> strings;
	
    /*! \class GravityTorquesManual
     * \brief Defines Orocos component for computation of joint torques
     * as a result of gravity. Note that only positive revolute joints are
     * supported however support of prismatic joints joints could be easily
     * implemented.
	 * 
     * A robot model should be provided that consists of frame
     * transformations that make sure the joint rotations are positive
     * and the frame transformations consist of a rotation around any axis
     * and a translation by a vector
     *
     * Every link that has a mass, should be provided with coordinates
     * of the location of the Centor of gravity using COGx, COGy, and COGz.
     * for every COG, considered as point mass, a kinematic chain is
     * generated with the tip frame in the COG
     *
     * A partial jacobian matrix is then calculated for each COG to the
     * base frame.
     *
     * Using partial_jacobian_.transpose() * *GravityWrench_. the gravity
     * joint torques of the ith COG is calculated summing to the total
     * output gravityTorques.
     *
     * iterators
     * i iterates over all the masses
     * j iterates over all the joints
     * k iterates over all DOF in cartesian space
     *
     * To Do:
     * Rewrite to parse urdf in stead of declaring parameters in ops file
	 */
	
	class GravityTorquesManual : 
	
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
    doubles rotation_angle;
    doubles translation_X;
    doubles translation_Y;
    doubles translation_Z;
	doubles masses;
    doubles COG_X;
    doubles COG_Y;
    doubles COG_Z;
    doubles GravityVector;

    // input variable
	doubles jointAngles;

	//variables
    ints mass_indexes;
    uint nrMasses;

    // Vectors of GravityWrenches, Robot Arm Chains and jacobian solvers. ith elemen of these vectors represent object for ith mass
    Eigen::VectorXd GravityWrenches[MAXJOINTS];
    KDL::Chain RobotArmChain[MAXJOINTS];
    KDL::ChainJntToJacSolver* jacobian_solver[MAXJOINTS];

	public:
	
	GravityTorquesManual(const std::string& name);
	~GravityTorquesManual();
	
	bool configureHook();
	bool startHook();
	void updateHook();

    doubles ComputeGravityTorques(KDL::JntArray q_current_);

	};
};
