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

#include "ARM_GravityTorques.hpp"

using namespace ARM;

GravityTorques::GravityTorques(const std::string& name)
	: TaskContext(name, PreOperational)
{
	// declaration of ports
	addEventPort("in", jointAnglesPort);
	addPort("out",gravityTorquesPort);

    // declaration of arm parameters and gravityvector
	addProperty( "nrJoints", nrJoints ).doc("An unsigned integer that specifies the number of degrees of freedom");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame. (array(0.0, 0.0, -9.81) for negative z direction)");
    addProperty( "joint_type", joint_type ).doc("");
    addProperty( "joint_axis", joint_axis ).doc("");
    addProperty( "rotation_axis", rotation_axis ).doc("");
    addProperty( "rotation_angle", rotation_angle ).doc("");
    addProperty( "translation_X", translation_X ).doc("");
    addProperty( "translation_Y", translation_Y ).doc("");
    addProperty( "translation_Z", translation_Z ).doc("");

	addProperty( "masses", masses ).doc("mass vector m stored in an Eigen::MatrixXd vector");
	addProperty( "COGx", COGx ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGy", COGy ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGz", COGz ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
    if (nrJoints >= MAXJOINTS) {
        log(Error)<<"ARM GravityTorques: Could not configure Gravity torques component: The number of joints " << nrJoints << " exceeds the maximum number of joints " << MAXJOINTS << "!"<<endlog();
        return false;
    }

    mass_indexes.assign(nrJoints,0);
    nrMasses = 0;
    printed = false;

	return true;
}

bool GravityTorques::startHook()
{
    // Parameter input checks
    if ( joint_type.size() != nrJoints || joint_axis.size() != nrJoints || rotation_angle.size() != nrJoints || rotation_axis.size() != nrJoints || translation_X.size() != nrJoints|| translation_Y.size() != nrJoints|| translation_Z.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized arm parameters!"<<endlog();
        return false;
    }

    // Parameter input checks
    if ( masses.size() != nrJoints || COGx.size() != nrJoints || COGy.size() != nrJoints || COGz.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized masses, COGx, COGy or COGz!"<<endlog();
        return false;
    }
    for (uint k = 0; k < 3; k++) {
        GravityWrenchGlobal(k) = GravityVector[k];
    }

    // For every non zero mass a gravity force is calculated.
    for (uint j = 0; j < nrJoints; j++) {
        if (masses[j] != 0.0) {
            mass_indexes[nrMasses] = j;
            nrMasses++;
        }
    }
    log(Warning)<<"ARM GravityTorques: Number of masses: [" << nrMasses  << "]" <<endlog();

    // construct chains and solvers
    for (uint i = 0; i < nrMasses; i++) {
        log(Warning)<<"ARM GravityTorques: Constructing Chain and Solver to link: [" << mass_indexes[i]  << "] and with mass: [" << masses[mass_indexes[i]] << "]" <<endlog();
        for (uint j = 0; j < mass_indexes[i]; j++) {

            KDL::Segment Segment_;
            KDL::Frame Frame_;

            // Determine Frame of fixed rotation and translation
            if (rotation_axis[j] == "X") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector(translation_X[j],translation_Y[j],translation_Z[j]));
            }
            else if (rotation_axis[j] == "Y") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector(translation_X[j],translation_Y[j],translation_Z[j]));
            }
            else if (rotation_axis[j] == "Z") {
                Frame_ = Frame(Rotation::RotX(rotation_angle[j]),Vector(translation_X[j],translation_Y[j],translation_Z[j]));
            }
            else if (rotation_axis[j] == "") {
                Frame_ = Frame(Vector(translation_X[j],translation_Y[j],translation_Z[j]));
            }
            else {
                log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: wrong rotation_axis for joint " << j << "!"<<endlog();
                return false;
            }

            // Construct Segment with joint, and the fixed rotation and translation of Frame_. Note that Prismatic joints are not (yet) supported.
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
            }
            else if (joint_type[j] == "P" ) {
                log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: Prismatic Joints are not yet supported!"<<endlog();
                return false;
            }
            else {
                log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: wrong joint type for joint " << j << "!"<<endlog();
                return false;
            }
            RobotArmChain[i].addSegment(Segment_);
        }

        // construct jacobian solver
        jacobian_solver[i] = new KDL::ChainJntToJacSolver(RobotArmChain[i]);
    }

    // Connection checks
    if ( !jointAnglesPort.connected() ){
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
        //return false;
    }
    if ( !gravityTorquesPort.connected() ){
        //log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: Outputport not connected!"<<endlog();
        //return false;
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

    // Joint Torques are calculated by multiplying the wrench for each link by the corresponding partial Jacobian
	for (uint i=0; i<nrMasses; i++) {

        //Determine Jacobian (KDL)        
        KDL::Jacobian partial_jacobian_KDL_(nrJoints);
        jacobian_solver[0]->JntToJac(q_current_, partial_jacobian_KDL_);

        //Convert Jacobian of type KDL to type Eigen::MatrixXd
        Eigen::MatrixXd partial_jacobian_(partial_jacobian_KDL_.rows(), partial_jacobian_KDL_.columns());
        for (uint jc=0; jc<partial_jacobian_KDL_.columns(); jc++) {
            for (uint jr=0; jr<partial_jacobian_KDL_.rows(); jr++) {
                partial_jacobian_(jr,jc) = partial_jacobian_KDL_(jr,jc);
                if (printed == false) {
                    //log(Warning) << "INDEX: [" << partial_jacobian_(jr,jc) << "]" <<endlog();
                }
            }
        }
        //printed = true;

        Eigen::VectorXd PartialGravityTorques_(nrJoints);

        // calculate gravityWrench
        KDL::Wrench GravityWrenchKDL_;
        KDL::Vector COG_translation;
        COG_translation(0) = COGx[mass_indexes[i]];
        COG_translation(1) = COGy[mass_indexes[i]];
        COG_translation(2) = COGz[mass_indexes[i]];
        GravityWrenchKDL_ = GravityWrenchGlobal;
        //GravityWrenchKDL_ = GravityWrenchGlobal.RefPoint(COG_translation);
        GravityWrenchKDL_ = GravityWrenchKDL_*masses[mass_indexes[i]];
        Eigen::VectorXd GravityWrench_(6);
        for (uint l=0; l<6; l++) {
            GravityWrench_(l) = 1.0;
        }

        // calculate partial gravity torque
        PartialGravityTorques_ = partial_jacobian_.transpose()*GravityWrench_;

        for (uint j=0; j<nrJoints; j++) {
            gravityTorques_[j] = PartialGravityTorques_(j);
        }
    }

    log(Warning) << "GravityTorques T: [" << gravityTorques_[0] << "," << gravityTorques_[1] << "," << gravityTorques_[2] << "," << gravityTorques_[3] << "," << gravityTorques_[4] << "," << gravityTorques_[5] << "," << gravityTorques_[6] << "]" <<endlog();

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)

