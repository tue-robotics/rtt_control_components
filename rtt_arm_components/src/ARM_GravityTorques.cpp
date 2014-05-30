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
    addProperty( "mass", m ).doc("mass vector m stored in an Eigen::MatrixXd vector");
	
    COG.setZero(3,nrJoints);
    addProperty( "COGx", COG(0) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
    addProperty( "COGy", COG(1) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
    addProperty( "COGz", COG(2) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
	return true;
}

bool GravityTorques::startHook()
{
    log(Warning)<<"ARM GravityTorques: StartHook complete "<<endlog();

    if ( !jointAnglesPort.connected() ){
        log(Error)<<"ARM GravityTorques: jointAnglesPort not connected!"<<endlog();
        return false;
    }
    if ( !gravityTorquesPort.connected() ){
        log(Warning)<<"ARM GravityTorques: Outputport not connected!"<<endlog();
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
    doubles gravityTorques_(nrJoints,0.0);
    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)

