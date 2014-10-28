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

#include "GravityTorquesManual.hpp"

using namespace ARM;

GravityTorquesManual::GravityTorquesManual(const std::string& name)
	: TaskContext(name, PreOperational)
{
	// declaration of ports
	addEventPort("in", jointAnglesPort);
    addPort("out",gravityTorquesPort);

    // declaration of arm parameters and gravityvector
    addProperty( "nrJoints", nrJoints ).doc("An unsigned integer that specifies the number of degrees of freedom");
    addProperty( "joint_type", joint_type ).doc("A vector of strings specifying the type of joint. R for revolute and P for prismatic (not yet supported)");
    addProperty( "joint_axis", joint_axis ).doc("A vector of strings specifying which axis the joint is defined to");
    addProperty( "rotation_axis", rotation_axis ).doc("A vector of strings specifying a rotation around a fixed rotation axis with the angle specified by rotation_angle");
    addProperty( "rotation_angle", rotation_angle ).doc("A vector of doubles specifying the amount of fixed rotation about the axis specified by rotation_axis");
    addProperty( "translation_X", translation_X ).doc("A vector of doubles specifying the translation around the X axis");
    addProperty( "translation_Y", translation_Y ).doc("A vector of doubles specifying the translation around the Y axis");
    addProperty( "translation_Z", translation_Z ).doc("A vector of doubles specifying the translation around the Z axis");
	addProperty( "masses", masses ).doc("mass vector m stored in an Eigen::MatrixXd vector");
    addProperty( "COG_X", COG_X ).doc("center of gravity coordinate vector x");
    addProperty( "COG_Y", COG_Y ).doc("center of gravity coordinate vector y");
    addProperty( "COG_Z", COG_Z ).doc("center of gravity coordinate vector z");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame. (array(0.0, 0.0, -9.81) for negative z direction)");
}

GravityTorquesManual::~GravityTorquesManual(){}

bool GravityTorquesManual::configureHook()
{
	
    //! Parameter input checks
    if (nrJoints >= MAXJOINTS) {
        log(Error)<<"ARM GravityTorquesManual: Could not configure Gravity torques component: The number of joints " << nrJoints << " exceeds the maximum number of joints " << MAXJOINTS << "!"<<endlog();
        return false;
    }

    if ( joint_type.size() != nrJoints || joint_axis.size() != nrJoints || rotation_angle.size() != nrJoints || rotation_axis.size() != nrJoints || translation_X.size() != nrJoints|| translation_Y.size() != nrJoints|| translation_Z.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component due to Wrongly sized arm parameters!"<<endlog();
        return false;
    }

    if ( masses.size() != nrJoints || COG_X.size() != nrJoints || COG_Y.size() != nrJoints || COG_Z.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component due to Wrongly sized masses, COG_X, COG_Y or COG_Z!"<<endlog();
        return false;
    }

    //! Count the number of masses, and store the indexes of these masses
    nrMasses = 0;
    for (uint j = 0; j < nrJoints; j++) {
        if (masses[j] != 0.0) {
            nrMasses++;
        }
    }
    mass_indexes.assign(nrMasses,0);
    int index = 0;
    for (uint j = 0; j < nrJoints; j++) {
        if (masses[j] != 0.0) {
            mass_indexes[index] = j;
            index++;
        }
    }
    log(Info)<<"ARM GravityTorquesManual: Number of masses: [" << nrMasses  << "]" <<endlog();

    //! Construct a vector consisting of a gravity wrench vector for each mass
    for (uint i = 0; i < nrMasses; i++) {
        GravityWrenches[i].resize(6);
        for (uint k=0; k<3; k++) {
            GravityWrenches[i](k) = GravityVector[k]*masses[mass_indexes[i]];
        }
    }

    //! Construct chains and solvers for each mass
    for (uint i = 0; i < nrMasses; i++) {
        log(Info)<<"ARM GravityTorquesManual: Constructing Chain and Solver to link: [" << mass_indexes[i]  << "] and with mass: [" << masses[mass_indexes[i]] << "]" <<endlog();
        for (uint j = 0; j < (mass_indexes[i]+1); j++) {

            KDL::Segment Segment_;
            KDL::Frame Frame_;
            KDL::Vector Vector_;
            string rotation;

            // Construct Vector of translation (if last segment, then frame is positioned in COG)
            if (j == mass_indexes[i]){
                Vector_ = Vector(COG_X[j],COG_Y[j],COG_Z[j]);
                rotation = "";
            }
            else {
                Vector_ = Vector(translation_X[j],translation_Y[j],translation_Z[j]);
            }

            // Construct Frame of fixed rotation and translation vector
            if (rotation_axis[j] == "X") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector_);
                
                rotation = "Rotation around X of ";
            }
            else if (rotation_axis[j] == "Y") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector_);
                rotation = "Rotation around Y of ";
            }
            else if (rotation_axis[j] == "Z") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector_);
                rotation = "Rotation around Z of ";
            }
            else if (rotation_axis[j] == "") {
                Frame_ = Frame(Vector_);
                rotation = "";
            }
            else {
                log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component: wrong rotation_axis for joint " << j << "!"<<endlog();
                return false;
            }

            // Construct Segment with joint and Frame_. Note that Prismatic joints are not (yet) supported.
            if (joint_type[j] == "R" ) {
                if (joint_axis[j] == "X") {
                    Segment_ = Segment(Joint(Joint::RotX),Frame_);
                }
                if (joint_axis[j] == "Y") {
                    Segment_ = Segment(Joint(Joint::RotY),Frame_);
                }
                if (joint_axis[j] == "Z") {
                    Segment_ = Segment(Joint(Joint::RotZ),Frame_);
                }
                if (joint_axis[j] == "-X") {
                    Segment_ = Segment(Joint(Joint::RotX,-1.0),Frame_);
                }
                if (joint_axis[j] == "-Y") {
                    Segment_ = Segment(Joint(Joint::RotY,-1.0),Frame_);
                }
                if (joint_axis[j] == "-Z") {
                    Segment_ = Segment(Joint(Joint::RotZ,-1.0),Frame_);
                }
            }
            else if (joint_type[j] == "P" ) {
                log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component: Prismatic Joints are not yet supported!"<<endlog();
                return false;
            }
            else {
                log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component: wrong joint type for joint " << j << "!"<<endlog();
                return false;
            }
            RobotArmChain[i].addSegment(Segment_);

            log(Info)<<"ARM GravityTorquesManual: Revolute joint around " << joint_axis[j] << ", " << rotation << rotation_angle[j] << ", Translate with [" << translation_X[j] << "," << translation_Y[j] << "," << translation_Z[j] << "]" <<endlog();
        }

        log(Info)<<"ARM GravityTorquesManual: Size of RobotArmChain is : " << RobotArmChain[i].segments.size() << "!"<<endlog();

        jacobian_solver[i] = new KDL::ChainJntToJacSolver(RobotArmChain[i]);
    }

    return true;
}

bool GravityTorquesManual::startHook()
{
	
    //! Connection checks
    if ( !jointAnglesPort.connected() ){
        log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
        return false;
    }
    if ( !gravityTorquesPort.connected() ){
        log(Error)<<"ARM GravityTorquesManual: Could not start Gravity torques component: Outputport not connected!"<<endlog();
        return false;
    }

    return true;
}

void GravityTorquesManual::updateHook()
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

doubles GravityTorquesManual::ComputeGravityTorques(KDL::JntArray q_current_)
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

ORO_CREATE_COMPONENT(ARM::GravityTorquesManual)
