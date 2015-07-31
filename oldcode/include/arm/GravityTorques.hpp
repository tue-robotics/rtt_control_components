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
	typedef vector<double> doubles;
    typedef vector<uint> ints;
    typedef vector<string> strings;
	
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
	 */
	
	class GravityTorques : 
	
	public RTT::TaskContext
	{
	private:
	
	InputPort<doubles> jointAnglesPort;
    OutputPort<doubles> gravityTorquesPort;
	
    // Properties
    string root_link_name;
    string tip_link_name;
    doubles GravityVector;

	//variables
    uint nrJoints;
    doubles masses;
    KDL::Chain kdl_chain;
    KDL::ChainJntToJacSolver* jacobian_solver[MAXJOINTS];
    std::vector < KDL::Frame > link_frames;         // [frame from mount to link 1, mount to link 2, ... ,mount to link nrJoints]
    std::vector < KDL::Vector > link_COGs;          // std vector of KDL Vectors. (for each joint, a vector from COG to the link center)
    KDL::Wrenches GravityWrenches_COG;              // gravity wrench in center of grav but wrt root frame
	public:
	
	GravityTorques(const std::string& name);
	~GravityTorques();
	
	bool configureHook();
	bool startHook();
	void updateHook();

    doubles ComputeGravityTorques(KDL::JntArray q_current_);

	};
};
