/*!
 * \author Max Baeten
 * \date May, 2014
 * \version 1.0
 * 
 * To Do:
 * - Fix Robot chain parsing
 * - Fix function computeGravityTorques
 * - Integrate frame tip for calculation of partial Jacobian
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
	GravityVector.setZero(3,1);
	GravityVector(2,0) = -9.81;
	
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
	GravityForces.setZero(3,nrMasses);
	for (uint i=0; i<nrMasses; i++) {
		GravityForces.col(i) << masses[mass_indexes[i]]*GravityVector;
	}

	// construct chain
	for (uint i = 0; i < nrJoints; i++) {
		//RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotZ(DH_alpha[i])),Frame(Rotation::RotZ(DH_theta[i])),Vector(DH_a[i],0.0,DH_d[i])));
		RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::RotZ(DH_alpha[i]))));
	}
	
	// construct jacobian solver
	jacobian_solver_ = new KDL::ChainJntToJacSolver(RobotArmChain);
	
	// Connection checks
	if ( !jointAnglesPort.connected() ){
		log(Error)<<"ARM GravityTorques: jointAnglesPort not connected!"<<endlog();
		//return false;
	}
	if ( !gravityTorquesPort.connected() ){
		log(Warning)<<"ARM GravityTorques: Outputport not connected!"<<endlog();
		//return false;
	}

	return true;
}

void GravityTorques::updateHook()
{
	// read jointAngles
	jointAngles.assign(nrJoints,0.0);
	jointAnglesPort.read(jointAngles);

	// jointAngles to KDL jointArray
	KDL::JntArray q_current_;
	for (uint i = 0; i < nrJoints; i++) {
		q_current_(i) = jointAngles[i];
	}
	// calculate gravityTorques
	doubles gravityTorques(nrJoints,0.0);
	gravityTorques = ComputeGravityTorques(q_current_);

	//write gravity torques
	gravityTorquesPort.write(gravityTorques);
}

Eigen::MatrixXd GravityTorques::ComputeJacobian(KDL::JntArray q_current_, int chain_tip_index)
{
	//jacobian_solver_->JntToJac(q_current, partial_jacobian);
	
	Eigen::MatrixXd partial_jacobian_;
	return partial_jacobian_;
}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current_)
{
	// For every Gravity force, The Joint Torques are calculated by multiplying the force by the corresponding Jacobian
	//doubles gravityTorques_(nrJoints,0.0);
	for (uint i=0; i<nrMasses; i++) {
		Eigen::MatrixXd partial_jacobian = ComputeJacobian(q_current_, mass_indexes[i]);
		//gravityTorques_ = gravityTorques_ + partial_jacobian * GravityForces.col(i);
	}
	doubles gravityTorques_(nrJoints,0.0);
	return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
