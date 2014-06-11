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
	
    /*! \class GravityTorques
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
     *
     *
     * To Do:
     * Rewrite to parse urdf in stead of declaring parameters in ops file
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
	
    KDL::Chain RobotArmChain[MAXJOINTS];
    KDL::ChainJntToJacSolver* jacobian_solver[MAXJOINTS];
    KDL::Wrench GravityWrenchGlobal;
	
	//variables
    ints mass_indexes;
	uint nrMasses;

	public:
	
	GravityTorques(const std::string& name);
	~GravityTorques();
	
	bool configureHook();
	bool startHook();
	void updateHook();
	
	doubles ComputeGravityTorques(KDL::JntArray q_current_);

	};
};
