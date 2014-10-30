/*!
 * \author Max Baeten
 * \date May, 2014
 * \version 1.0
 */
 
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>
#include "GravityTorques.hpp"

using namespace std;
using namespace KDL;
using namespace ARM;

GravityTorques::GravityTorques(const std::string& name)
	: TaskContext(name, PreOperational)
{
	// declaration of ports
	addEventPort("in", jointAnglesPort);
    addPort("out",gravityTorquesPort); 
    
    addProperty( "RootLinkName", root_link_name ).doc("Name of the Base link frame of the manipulator");
    addProperty( "TipLinkName", tip_link_name ).doc("Name of the Tip link frame of the manipulator");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame");
}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
    // Fetch robot_model_ from parameter server (URDF robot model)
    ros::NodeHandle n("~");
    std::string urdf_xml_default = "/amigo/robot_description";
    n.param("urdf_xml",urdf_xml,urdf_xml_default);
    n.searchParam(urdf_xml,full_urdf_xml);
    ROS_INFO("Reading xml file from parameter server");
    std::string urdf_model_string;
    if (!n.getParam(full_urdf_xml,urdf_model_string)){
        log(Error)<<"ARM GravityTorques: Could not load the xml from parameter server: " <<full_urdf_xml << "!" <<endlog();
        return false;
    }
     
    // Construct KDL::Tree from URDF model
    if (!kdl_parser::treeFromString(urdf_model_string, kdl_tree_)) {
        log(Error)<<"ARM GravityTorques:Could not construct tree object from URDF model!" <<endlog();
        return false;
    }
        
    // Extract whole KDL::Chain from robot tree
    if (!kdl_tree_.getChain(root_link_name, tip_link_name, kdl_chain_)) {
        log(Error)<<"ARM GravityTorques: Could not construct chain object from " << root_link_name << " to " << tip_link_name << "!" <<endlog();
        return false;
    }

    // init
    nrJoints = kdl_chain_.getNrOfSegments();
    nrMasses = 0;
    masses.assign(nrJoints,0.0);
    mass_indexes.assign(nrJoints,0);
    KDL::Vector KDLZeroVec;
    KDLZeroVec.Zero();

    // Print joint layout (For debugging purposes)
    for (uint j=0; j<nrJoints; j++) {

        KDL::Segment JointSegment = kdl_chain_.getSegment(j);
        KDL::Frame JointTipFrame = JointSegment.getFrameToTip();
        KDL::RigidBodyInertia JointRBI = JointSegment.getInertia();
        KDL::Vector JointCOG = JointRBI.getCOG();

        log(Warning)<<"ARM GravityTorques: Parsing joint "<< j+1 <<" with center of joint   : [" << JointTipFrame.p[0] << ",\t" << JointTipFrame.p[1] << ",\t" << JointTipFrame.p[2] << "]" <<endlog();
    }

    // Check for every joint if there is a link attached
    for (uint j=0; j<nrJoints; j++) {

        KDL::Segment JointSegment = kdl_chain_.getSegment(j);
        KDL::Frame JointTipFrame = JointSegment.getFrameToTip();
        KDL::RigidBodyInertia JointRBI = JointSegment.getInertia();
        KDL::Vector JointCOG = JointRBI.getCOG();

        log(Warning)<<"ARM GravityTorques: Parsing joint "<< j+1 <<" with center of gravity : [" << JointCOG[0] << ",\t" << JointCOG[1] << ",\t" << JointCOG[2] << "] and Mass: " << JointRBI.getMass() << "!" <<endlog();

        //log(Warning)<<"ARM GravityTorques: Parsed joint "<< j+1 <<" with center : [" << JointTipFrame.p[0] << ",\t" << JointTipFrame.p[1] << ",\t" << JointTipFrame.p[2] << "]!" <<endlog();

        if (JointCOG != KDLZeroVec ) {
            masses[j] = JointRBI.getMass();
            mass_indexes[nrMasses] = j;

            // Create chain
            KDL::Chain link_chain_;
            if (!kdl_tree_.getChain(root_link_name, JointSegment.getName(), link_chain_)) {
                log(Error)<<"ARM GravityTorques: Could not construct chain object from " << root_link_name << " to " << JointSegment.getName() << "!" <<endlog();
                return false;
            }

            // Create solver
            jacobian_solver[nrMasses] = new KDL::ChainJntToJacSolver(link_chain_);
            //log(Warning)<<"ARM GravityTorques: Solver created for Joint "<< j+1 <<" with COG: [" << JointCOG[0] << ",\t" << JointCOG[1] << ",\t" << JointCOG[2] << "] and Mass: " << masses[j] << "!" <<endlog();

            nrMasses++;
        }
    }

    //! Construct a vector consisting of a gravity wrench vector for each mass
    for (uint m = 0; m < nrMasses; m++) {
        GravityWrenches[m].resize(6);
        for (uint i=0; i<3; i++) {
            GravityWrenches[m](i) = GravityVector[i]*masses[mass_indexes[m]];
            GravityWrenches[m](i+3) = 0.0;
        }
        log(Warning)<<"ARM GravityTorques: Created Gravity Wrench [" << GravityWrenches[m](0) << "," << GravityWrenches[m](1) << "," << GravityWrenches[m](2) << "," << GravityWrenches[m](3) << "," << GravityWrenches[m](4) << "," << GravityWrenches[m](5) << "]" <<endlog();
    }

	if (mass_indexes.size() >= 2 ) { 
		log(Warning)<<"ARM GravityTorques: Mass indexes are [" << mass_indexes[0] << "," << mass_indexes[1] << "," << mass_indexes[2] << "]" <<endlog(); 
	}

    return true;
}

bool GravityTorques::startHook()
{
    //! Connection checks
    if(!jointAnglesPort.connected()){
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
        return false;
    }
    if(!gravityTorquesPort.connected()){
        log(Info)<<"ARM GravityTorques: Outputport not connected!"<<endlog();
    }

    return true;
}

void GravityTorques::updateHook()
{
    // read jointAngles
    jointAngles.assign(nrJoints,0.0);

    if ( jointAnglesPort.read(jointAngles) == NewData ) {
        // jointAngles to KDL jointArray
        KDL::JntArray q_current_(nrJoints);
        for (uint j = 0; j < nrJoints; j++) {
            q_current_(j) = jointAngles[j];
        }

        // calculate gravityTorques
        doubles gravityTorques(nrJoints,0.0);
        gravityTorques = ComputeGravityTorques(q_current_);

        //write gravity torques
        gravityTorquesPort.write(gravityTorques);        
    }
}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current_)
{
    doubles gravityTorques_(nrJoints,0.0);

    for (uint m=0; m<nrMasses; m++) {

        //! Convert KDL::JntArray to size of partial Jacobian
        KDL::JntArray q_current_partial_;
        q_current_partial_.resize(mass_indexes[m]+1);
        for (uint j=0; j<(mass_indexes[m]+1); j++) {
            q_current_partial_(j) = q_current_(j);
        }

        //! Determine Jacobian (KDL)
        KDL::Jacobian partial_jacobian_KDL_(mass_indexes[m]+1);
        jacobian_solver[m]->JntToJac(q_current_partial_, partial_jacobian_KDL_);

        //Convert Jacobian of type KDL to type Eigen::MatrixXd
        Eigen::MatrixXd partial_jacobian_(partial_jacobian_KDL_.rows(), partial_jacobian_KDL_.columns());
        for (uint i=0; i<partial_jacobian_KDL_.columns(); i++) {
            for (uint k=0; k<partial_jacobian_KDL_.rows(); k++) {
                partial_jacobian_(k,i) = partial_jacobian_KDL_(k,i);
            }
        }

        //! Calculate partial gravity torque
        Eigen::VectorXd PartialGravityTorques_ = Eigen::VectorXd::Zero((mass_indexes[m]+1));
        PartialGravityTorques_ = partial_jacobian_.transpose()*GravityWrenches[m];

        //! convert partial gravity torque to doubles and add all partial gravity torques together
        for (uint j=0; j<(mass_indexes[m]+1); j++) {
            gravityTorques_[j] += PartialGravityTorques_(j);
        }
    }

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
