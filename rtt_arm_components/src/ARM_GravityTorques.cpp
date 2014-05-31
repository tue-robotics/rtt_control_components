/*!
 * \author Max Baeten
 * \date May, 2014
 * \version 1.0 
 */
 
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>

#include "ARM_GravityTorques.hpp"

using namespace std;
using namespace RTT;
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
	
	// assinging robot segments to RobotArmChain // to do obtain type of joint and vector from ops file	
	RobotArmChain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));

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
	doubles jointAngles(nrJoints,0.0);
	jointAnglesPort.read(jointAngles);

	// calculate gravityTorques
	doubles gravityTorques(nrJoints,0.0);
	gravityTorques = ComputeGravityTorques();

	//write gravity torques
	gravityTorquesPort.write(gravityTorques);
}

Eigen::MatrixXd GravityTorques::ComputeJacobian()
{
	Eigen::MatrixXd J_;
	return J_;
}

doubles GravityTorques::ComputeGravityTorques()
{
	// For every Gravity force, The Joint Torques are calculated by multiplying the force by the corresponding Jacobian
	doubles gravityTorques_(nrJoints,0.0);
	return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)

