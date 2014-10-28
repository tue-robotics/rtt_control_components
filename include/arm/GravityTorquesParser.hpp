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
#include <kdl_parser/kdl_parser.hpp>

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
     * A robot model should be provided that consists of an URDF model, 
     * this model is then parsed to a kdl tree out of which a KDL chain 
     * is extracted.
     *
     * To Do:
     * Rewrite to parse urdf in stead of declaring parameters in ops file
	 */
	
	class GravityTorquesParser : 
	
	public RTT::TaskContext
	{
	private:
	
	InputPort<doubles> jointAnglesPort;
    OutputPort<doubles> gravityTorquesPort;
	
	//Properties
	std::string base_link_name;
    std::string tip_link_name; 
	std::string urdf_xml;
	std::string full_urdf_xml;
	doubles masses;
    doubles GravityVector;

    // input variable
	doubles jointAngles;

	//variables
    ints mass_indexes;
    uint nrJoints;
    uint nrMasses;

    // Vectors of GravityWrenches, Robot Arm Chains and jacobian solvers. ith elemen of these vectors represent object for ith mass
    Eigen::VectorXd GravityWrenches[MAXJOINTS];
    KDL::Chain RobotArmChain[MAXJOINTS];
    KDL::ChainJntToJacSolver* jacobian_solver[MAXJOINTS];
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    urdf::Model robot_model_;


	public:
	
	GravityTorquesParser(const std::string& name);
	~GravityTorquesParser();
	
	bool configureHook();
	bool startHook();
	void updateHook();

    doubles ComputeGravityTorques(KDL::JntArray q_current_);

	};
};
