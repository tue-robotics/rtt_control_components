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
    
    addProperty( "RootLinkName", root_link_name ).doc("Name of the frame at root of the bodypart");
    addProperty( "TipLinkName", tip_link_name ).doc("Name of the frame at the end link of the bodypart");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame");
}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
    // Fetch robot_model_ from parameter server (URDF robot model)
    std::string urdf_xml_default = "/amigo/robot_description";
    string urdf_xml;
    string full_urdf_xml;
    ros::NodeHandle n("~");
    n.param("urdf_xml",urdf_xml,urdf_xml_default);
    n.searchParam(urdf_xml,full_urdf_xml);
    ROS_INFO("Reading xml file from parameter server");
    std::string urdf_model_string;
    if (!n.getParam(full_urdf_xml,urdf_model_string)){
        log(Error)<<"ARM GravityTorques: Could not load the xml from parameter server: " <<full_urdf_xml << "!" <<endlog();
        return false;
    }

    // Construct KDL::Tree from URDF model
    KDL::Tree KDL_tree;
    if (!kdl_parser::treeFromString(urdf_model_string, KDL_tree)) {
        log(Error)<<"ARM GravityTorques:Could not construct tree object from URDF model!" <<endlog();
        return false;
    }
        
    // Extract whole KDL::Chain from robot tree
    if (!KDL_tree.getChain(root_link_name, tip_link_name, kdl_chain)) {
        log(Error)<<"ARM GravityTorques: Could not construct chain object from " << root_link_name << " to " << tip_link_name << "!" <<endlog();
        return false;
    }

    // Initialisation
    double total_mass = 0.0;
    nrJoints = kdl_chain.getNrOfSegments();
    masses.assign(nrJoints,0.0);
    GravityWrenches_COG.resize(nrJoints);
    link_frames.resize(nrJoints);
    link_COGs.resize(nrJoints);

    for (uint j=0; j<nrJoints; j++) {

        // init
        GravityWrenches_COG[j].Zero();
        link_COGs[j].Zero();

        // Get mass
        masses[j] = kdl_chain.getSegment(j).getInertia().getMass();
        total_mass += masses[j];

        KDL::Frame JointTipFrame = kdl_chain.getSegment(j).getFrameToTip();
        log(Warning)<<"ARM GravityTorques: Parsed joint "<< j+1 <<" with center : [" << JointTipFrame.p[0] << ",\t" << JointTipFrame.p[1] << ",\t" << JointTipFrame.p[2] << "]!" <<endlog();

        // Create COG vectors
        link_COGs[j] = kdl_chain.getSegment(j).getInertia().getCOG();
        log(Warning)<<"ARM GravityTorques: Added Joint " << j+1 << " with mass : " << masses[j] << " kg and with COG [" << link_COGs[j].x() << "," << link_COGs[j].y() << "," << link_COGs[j].z() << "]" <<endlog();
        link_COGs[j].ReverseSign();

        // Create chain
        KDL::Chain link_chain;
        if (!KDL_tree.getChain(root_link_name, kdl_chain.getSegment(j).getName(), link_chain)) {
            log(Error)<<"ARM GravityTorques: Could not construct chain object from " << root_link_name << " to " << kdl_chain.getSegment(j).getName() << "!" <<endlog();
            return false;
        }

        // Create solver
        jacobian_solver[j] = new KDL::ChainJntToJacSolver(link_chain);

        // Create gravity wrench
        for (uint i=0; i<3; i++) {
            KDL::Vector force(GravityVector[0]*masses[j], GravityVector[1]*masses[j], GravityVector[2]*masses[j]);
            GravityWrenches_COG[j] = KDL::Wrench(force, KDL::Vector::Zero());
        }
        log(Warning)<<"ARM GravityTorques: Created Gravity Wrench root [" << GravityWrenches_COG[j](0) << "," << GravityWrenches_COG[j](1) << "," << GravityWrenches_COG[j](2) << "," << GravityWrenches_COG[j](3) << "," << GravityWrenches_COG[j](4) << "," << GravityWrenches_COG[j](5) << "]" <<endlog();

    }
    log(Warning)<<"ARM GravityTorques: Total Mass: " << total_mass << " kg. " <<endlog();

    // to do: add check wether the gravity vector is aligned with the base frame vector

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
    doubles jointAngles(nrJoints,0.0);
    if ( jointAnglesPort.read(jointAngles) == NewData ) {
		
        // Tranform jointAngles from doubles to KDL::jointArray
        KDL::JntArray q_current(nrJoints);
        for (uint j = 0; j < nrJoints; j++) {
            q_current(j) = jointAngles[j];
        }

        // calculate gravityTorques
        doubles gravityTorques(nrJoints,0.0);
        gravityTorques = ComputeGravityTorques(q_current);

        //write gravity torques
        gravityTorquesPort.write(gravityTorques);        
    }
}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current)
{
    log(Info)<<"ARM GravityTorques: q = [" << q_current(0) << "," << q_current(1) << "," << q_current(2) << "," << q_current(3) << "," << q_current(4) << "," << q_current(5) << "," << q_current(6) << "]" << endlog();

    // Calculate Partial Jacobian, and
    doubles gravityTorques(nrJoints,0.0);
    for (uint j=0; j<nrJoints; j++) {

        // Calculate frames of joints from latest postions expressed in root frame
        link_frames[j] = kdl_chain.getSegment(j).pose(q_current(j));
        if (j>0) {
            link_frames[j] = link_frames[j-1]*link_frames[j];
        }

        //! Calculate Partial Jacobian
        KDL::JntArray q_currentpartial(j+1);
        KDL::Jacobian partial_jacobian(j+1);
        for (uint jj=0; jj<(j+1); jj++) {
            q_currentpartial(jj) = q_current(jj);
        }
        jacobian_solver[j]->JntToJac(q_currentpartial, partial_jacobian);

        //! Calculate GravityWrench
        KDL::Rotation R_rl = link_frames[j].M;
        KDL::Vector link_COG_r = R_rl*link_COGs[j];
        KDL::Wrench GravityWrenches_COL = GravityWrenches_COG[j].RefPoint(link_COG_r);

        //! Convert KDL::Wrench GravityWrench to Eigen::Vector
        Eigen::VectorXd GravityWrench(6);
        for (uint n=0; n<6; n++) {
            GravityWrench(n) = GravityWrenches_COL(n);
        }



        if (link_COG_r.Norm() > 0.001 ) {
            log(Info)<<"ARM GravityTorques: For Joint " << j+1 << " The link_COGs[j]                = [" << link_COGs[j].x() << "," << link_COGs[j].y() << "," << link_COGs[j].z() << "]" <<endlog();
            log(Info)<<"ARM GravityTorques: For Joint " << j+1 << " The link_COGs[j] transformed to = [" << link_COG_r.x() << "," << link_COG_r.y() << "," << link_COG_r.z() << "]" <<endlog();
            log(Info)<<"ARM GravityTorques: For Joint " << j+1 << " The start Gravity Wrench is     = [" << GravityWrenches_COG[j](0) << ", " << GravityWrenches_COG[j](1) << ", " << GravityWrenches_COG[j](2) << ", " << GravityWrenches_COG[j](3) << ", " << GravityWrenches_COG[j](4) << ", " << GravityWrenches_COG[j](5) << "]" <<endlog();
            log(Info)<<"ARM GravityTorques: For Joint " << j+1 << " The updated Gravity Wrench is   = [" << GravityWrenches_COL(0) << ", " << GravityWrenches_COL(1) << ", " << GravityWrenches_COL(2) << ", " << GravityWrenches_COL(3) << ", " << GravityWrenches_COL(4) << ", " << GravityWrenches_COL(5) << "] \n " <<endlog();
        }

        // Calculate Gravity Torques
        Eigen::VectorXd PartialGravityTorques = Eigen::VectorXd::Zero(j+1);
        PartialGravityTorques = partial_jacobian.data.transpose()*GravityWrench;

        //! convert partial gravity torque to doubles and add all partial gravity torques together
        for (uint jj=0; jj<(j+1); jj++) {
            gravityTorques[jj] += PartialGravityTorques(jj);
        }
    }

    return gravityTorques;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
