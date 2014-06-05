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
	
	// declaration of DH parameters, Mass vector and COG
	addProperty( "nrJoints", nrJoints ).doc("An unsigned integer that specifies the number of degrees of freedom");
	addProperty( "a", DH_a ).doc("An Eigen::MatrixXd vector containing DH parameters a");
	addProperty( "d", DH_d ).doc("An Eigen::MatrixXd vector containing DH parameters d");
	addProperty( "alpha", DH_alpha ).doc("An Eigen::MatrixXd vector containing DH parameters alpha");
	addProperty( "theta", DH_theta ).doc("An Eigen::MatrixXd vector containing DH parameters theta");
	addProperty( "masses", masses ).doc("mass vector m stored in an Eigen::MatrixXd vector");
	addProperty( "COGx", COGx ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGy", COGy ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGz", COGz ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
	GravityWrench.setZero(6,1);
    GravityWrench(0) = -10.0;
	
	return true;
}

bool GravityTorques::startHook()
{
	// For every non zero mass a gravity force is calculated.
	nrMasses = 0;
	for (uint i = 0; i < nrJoints; i++) {
		if (masses[i] != 0.0) {
			nrMasses++;
			mass_indexes.resize(nrMasses);
			mass_indexes[nrMasses-1] = i;
		}
	}
	
	// Construct Gravity Wrenches
	GravityWrenches.setZero(6,nrMasses);
	for (uint i=0; i<nrMasses; i++) {
		GravityWrenches.col(i) << masses[mass_indexes[i]]*GravityWrench;
	}

	// construct chain
	for (uint i = 0; i < nrJoints; i++) {
		RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotX(DH_alpha[i]),Vector(DH_a[i],0.0,DH_d[i])) ) );
	}
	
	// construct jacobian solver
    jacobian_solver = new KDL::ChainJntToJacSolver(RobotArmChain);
	
	// Connection checks
	if ( !jointAnglesPort.connected() ){
        log(Error)<<"ARM GravityTorques: jointAnglesPort not connected!"<<endlog();
        //return false;
	}
    if ( !gravityTorquesPort.connected() ){
        log(Warning)<<"ARM GravityTorques: Outputport not connected!"<<endlog();
		//return false;
    }

    log(Warning)<<"ARM GravityTorques: Number of masses found: [" << nrMasses  << "]" <<endlog();

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

    log(Warning) << "GravityTorques: [" << gravityTorques[0] << "," << gravityTorques[1] << "," << gravityTorques[2] << "," << gravityTorques[3] << "," << gravityTorques[4] << "," << gravityTorques[5] << "," << gravityTorques[6] << "]" <<endlog();
}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current_)
{
    doubles gravityTorques_(nrJoints,0.0);

    // Joint Torques are calculated by multiplying the wrench for each link by the corresponding partial Jacobian
	for (uint i=0; i<nrMasses; i++) {

        //Determine Jacobian (KDL)        
        KDL::Jacobian partial_jacobian_KDL_(nrJoints);
        jacobian_solver->JntToJac(q_current_, partial_jacobian_KDL_);

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
	}

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
