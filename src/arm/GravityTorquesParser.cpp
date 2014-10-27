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
#include "GravityTorquesParser.hpp"

using namespace std;
using namespace KDL;
using namespace ARM;

GravityTorquesParser::GravityTorquesParser(const std::string& name)
	: TaskContext(name, PreOperational)
{
	// declaration of ports
	addEventPort("in", jointAnglesPort);
    addPort("out",gravityTorquesPort);
    
    addProperty( "BaseLinkName", base_link_name ).doc("Name of the Base link frame of the manipulator");
    addProperty( "TipLinkName", tip_link_name ).doc("Name of the Tip link frame of the manipulator");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame");
}

GravityTorquesParser::~GravityTorquesParser(){}

bool GravityTorquesParser::configureHook()
{
	
    // Fetch robot_model_ from parameter server (URDF robot model)
    ros::NodeHandle n("~");
    std::string urdf_xml_default = "/amigo/robot_description";
    n.param("urdf_xml",urdf_xml,urdf_xml_default);
    n.searchParam(urdf_xml,full_urdf_xml);
    ROS_INFO("Reading xml file from parameter server");
    std::string urdf_model_string;
    if (!n.getParam(full_urdf_xml,urdf_model_string)){
        log(Error)<<"ARM GravityTorquesParser: Could not load the xml from parameter server: " <<full_urdf_xml << "!" <<endlog();
        return false;
    }
     
    // Construct KDL::Tree from URDF model
    if (!kdl_parser::treeFromString(urdf_model_string, kdl_tree_)) {
        log(Error)<<"ARM GravityTorquesParser:Could not construct tree object from URDF model!" <<endlog();
        return false;
    }
    log(Warning)<<"ARM GravityTorquesParser: constructed tree object from URDF model :)" <<endlog();
        
    // Extract KDL::Chain from robot tree
    if (!kdl_tree_.getChain(base_link_name, tip_link_name, kdl_chain_)) {
        log(Error)<<"ARM GravityTorquesParser: Could not construct chain object from " << base_link_name << " to " << tip_link_name << "!" <<endlog();
        return false;
    }
    log(Warning)<<"ARM GravityTorquesParser: constructed chain object from " << base_link_name << " to " << tip_link_name << " :) :) :) !" <<endlog();

    return true;
    }

bool GravityTorquesParser::startHook()
{
	
    //! Connection checks
    if(!jointAnglesPort.connected()){
        log(Error)<<"ARM GravityTorquesParser: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
        return false;
    }
    if(!gravityTorquesPort.connected()){
        log(Error)<<"ARM GravityTorquesParser: Could not start Gravity torques component: Outputport not connected!"<<endlog();
        return false;
    }

    return true;
}

void GravityTorquesParser::updateHook()
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

doubles GravityTorquesParser::ComputeGravityTorques(KDL::JntArray q_current_)
{
    doubles gravityTorques_(nrJoints,0.0);

    for (uint i=0; i<nrMasses; i++) {

        //! Convert KDL::JntArray to size of partial Jacobian
        KDL::JntArray q_current_partial_;
        q_current_partial_.resize(mass_indexes[i]+1);
        for (uint j=0; j<(mass_indexes[i]+1); j++) {
            q_current_partial_(j) = q_current_(j);
        }

        //! Determine Jacobian (KDL)
        KDL::Jacobian partial_jacobian_KDL_(mass_indexes[i]+1);
        jacobian_solver[i]->JntToJac(q_current_partial_, partial_jacobian_KDL_);

        //Convert Jacobian of type KDL to type Eigen::MatrixXd
        Eigen::MatrixXd partial_jacobian_(partial_jacobian_KDL_.rows(), partial_jacobian_KDL_.columns());
        for (uint j=0; j<partial_jacobian_KDL_.columns(); j++) {
            for (uint k=0; k<partial_jacobian_KDL_.rows(); k++) {
                partial_jacobian_(k,j) = partial_jacobian_KDL_(k,j);
            }
        }

        //! Calculate partial gravity torque
        Eigen::VectorXd PartialGravityTorques_ = Eigen::VectorXd::Zero((mass_indexes[i]+1));
        PartialGravityTorques_ = partial_jacobian_.transpose()*GravityWrenches[i];

        //! convert partial gravity torque to doubles and add all partial gravity torques together
        for (uint j=0; j<(mass_indexes[i]+1); j++) {
            gravityTorques_[j] += PartialGravityTorques_(j);
        }
    }

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorquesParser)
