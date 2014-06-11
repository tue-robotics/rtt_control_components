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
    addProperty( "rotation_angle", rotation_angle ).doc("");
    addProperty( "rotation_axis", rotation_axis ).doc("");
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
    // construct gravity wrench from GravityVector
	GravityWrench.setZero(6,1);
    for (uint i = 0; i < 3; i++) {
        GravityWrench(i) = GravityVector[i];
    }

    printed = false;
	
	return true;
}

bool GravityTorques::startHook()
{
//    // Parameter input checks
//    if ( DH_a.size() != nrJoints || DH_d.size() != nrJoints || DH_alpha.size() != nrJoints || DH_theta.size() != nrJoints || COGx.size() != nrJoints || COGy.size() != nrJoints || COGz.size() != nrJoints || masses.size() != nrJoints ) {
//        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized DH parameters, masses or COG!"<<endlog();
//        return false;
//    }

//    if ( rot_X.size() != nrJoints || rot_Z.size() != nrJoints ) {
//        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized rot_X or rot_Z parameters!"<<endlog();
//    }
//    else {
//        for (uint i = 0; i < nrJoints; i++) {
//            if ( rot_X[i] + rot_Z[i] != 1.0) {
//                log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to erroneous input of rot_X or rot_Z"<<endlog();
//                log(Error)<<"ARM GravityTorques: Please make sure that either: rot_X[" << i << "], or rot_Z[" << i << "] is equal to 1.0." <<endlog();
//                return false;
//            }
//        }
//    }

//	// For every non zero mass a gravity force is calculated.
//	nrMasses = 0;
//	for (uint i = 0; i < nrJoints; i++) {
//		if (masses[i] != 0.0) {
//			nrMasses++;
//			mass_indexes.resize(nrMasses);
//			mass_indexes[nrMasses-1] = i;
//		}
//	}
//    log(Warning)<<"ARM GravityTorques: Number of masses: [" << nrMasses  << "]" <<endlog();
	
//	// Construct Gravity Wrenches
//	GravityWrenches.setZero(6,nrMasses);
//	for (uint i=0; i<nrMasses; i++) {
//		GravityWrenches.col(i) << masses[mass_indexes[i]]*GravityWrench;
//    }

//    // construct chain
//	for (uint i = 0; i < nrJoints; i++) {
//        if (rot_X[i] == 1.0 ) {
//            if (DH_alpha[i] != 0.0) {
//                RobotArmChain.addSegment(Segment(Joint(Joint::RotX),Frame(Rotation::RotX(DH_alpha[i]),Vector(DH_a[i],0.0,DH_d[i])) ) );
//            }
//            else {
//                RobotArmChain.addSegment(Segment(Joint(Joint::RotX),Frame(Rotation::RotZ(DH_theta[i]),Vector(DH_a[i],0.0,DH_d[i])) ) );
//            }
//        }
//        else if (rot_Z[i] == 1.0 ) {
//            if (DH_alpha[i] != 0.0) {
//                RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotX(DH_alpha[i]),Vector(DH_a[i],0.0,DH_d[i])) ) );
//            }
//            else {
//                RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotZ(DH_theta[i]),Vector(DH_a[i],0.0,DH_d[i])) ) );
//            }
//        }
//        else {
//            log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: A value unequal to 1.0 was found in rot_Z and rot_X"<<endlog();
//            return false;
//        }
//    }

//	// construct jacobian solver
//    jacobian_solver = new KDL::ChainJntToJacSolver(RobotArmChain);
	
//	// Connection checks
//	if ( !jointAnglesPort.connected() ){
//        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
//        return false;
//	}
//    if ( !gravityTorquesPort.connected() ){
//        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: Outputport not connected!"<<endlog();
//		//return false;
//    }

    return true;
}

void GravityTorques::updateHook()
{
	// read jointAngles
	jointAngles.assign(nrJoints,0.0);
	jointAnglesPort.read(jointAngles);

	// jointAngles to KDL jointArray
	KDL::JntArray q_current_(nrJoints);
	for (uint i = 0; i < nrJoints; i++) {
		q_current_(i) = jointAngles[i];
	}
	
	// calculate gravityTorques
	doubles gravityTorques(nrJoints,0.0);
    gravityTorques = ComputeGravityTorques(q_current_);

	//write gravity torques
	gravityTorquesPort.write(gravityTorques);

}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current_)
{
    doubles gravityTorques_(nrJoints,0.0);

    // Joint Torques are calculated by multiplying the wrench for each link by the corresponding partial Jacobian
	for (uint i=0; i<nrMasses; i++) {

        //Determine Jacobian (KDL)        
        KDL::Jacobian partial_jacobian_KDL_(nrJoints);
        jacobian_solver->JntToJac(q_current_, partial_jacobian_KDL_);

        if ( printed == false ) {
            log(Warning) << "JAC1: [" << partial_jacobian_KDL_(0,0) << ", " << partial_jacobian_KDL_(0,1) << ", " << partial_jacobian_KDL_(0,2) << ", " << partial_jacobian_KDL_(0,3) << ", " << partial_jacobian_KDL_(0,4) << ", " << partial_jacobian_KDL_(0,5) << ", " << partial_jacobian_KDL_(0,6) << "]" <<endlog();
            log(Warning) << "JAC2: [" << partial_jacobian_KDL_(1,0) << ", " << partial_jacobian_KDL_(1,1) << ", " << partial_jacobian_KDL_(1,2) << ", " << partial_jacobian_KDL_(1,3) << ", " << partial_jacobian_KDL_(1,4) << ", " << partial_jacobian_KDL_(1,5) << ", " << partial_jacobian_KDL_(1,6) << "]" <<endlog();
            log(Warning) << "JAC3: [" << partial_jacobian_KDL_(2,0) << ", " << partial_jacobian_KDL_(2,1) << ", " << partial_jacobian_KDL_(2,2) << ", " << partial_jacobian_KDL_(2,3) << ", " << partial_jacobian_KDL_(2,4) << ", " << partial_jacobian_KDL_(2,5) << ", " << partial_jacobian_KDL_(2,6) << "]" <<endlog();
            log(Warning) << "JAC4: [" << partial_jacobian_KDL_(3,0) << ", " << partial_jacobian_KDL_(3,1) << ", " << partial_jacobian_KDL_(3,2) << ", " << partial_jacobian_KDL_(3,3) << ", " << partial_jacobian_KDL_(3,4) << ", " << partial_jacobian_KDL_(3,5) << ", " << partial_jacobian_KDL_(3,6) << "]" <<endlog();
            log(Warning) << "JAC5: [" << partial_jacobian_KDL_(4,0) << ", " << partial_jacobian_KDL_(4,1) << ", " << partial_jacobian_KDL_(4,2) << ", " << partial_jacobian_KDL_(4,3) << ", " << partial_jacobian_KDL_(4,4) << ", " << partial_jacobian_KDL_(4,5) << ", " << partial_jacobian_KDL_(4,6) << "]" <<endlog();
            log(Warning) << "JAC6: [" << partial_jacobian_KDL_(5,0) << ", " << partial_jacobian_KDL_(5,1) << ", " << partial_jacobian_KDL_(5,2) << ", " << partial_jacobian_KDL_(5,3) << ", " << partial_jacobian_KDL_(5,4) << ", " << partial_jacobian_KDL_(5,5) << ", " << partial_jacobian_KDL_(5,6) << "]" <<endlog();
            printed = true;
        }

        //Convert Jacobian of type KDL to type Eigen::MatrixXd
        Eigen::MatrixXd partial_jacobian_(partial_jacobian_KDL_.rows(), partial_jacobian_KDL_.columns());
        for (uint jc=0; jc<partial_jacobian_KDL_.columns(); jc++) {
            for (uint jr=0; jr<partial_jacobian_KDL_.rows(); jr++) {
                partial_jacobian_(jr,jc) = partial_jacobian_KDL_(jr,jc);
            }
        }
        Eigen::VectorXd PartialGravityTorques_(nrJoints);
        PartialGravityTorques_ = partial_jacobian_.transpose() * GravityWrenches.col(i);

        for (uint j=0; j<nrJoints; j++) {
            gravityTorques_[j] += PartialGravityTorques_(j);
        }

        log(Warning) << "GravityTorques: [" << gravityTorques_[0] << "," << gravityTorques_[1] << "," << gravityTorques_[2] << "," << gravityTorques_[3] << "," << gravityTorques_[4] << "," << gravityTorques_[5] << "," << gravityTorques_[6] << "]" <<endlog();

	}

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
