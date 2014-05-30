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
	grav.setZero(3,1);
	
	// declaration of ports
	addEventPort("in", jointAnglesPort);
	addPort("out",gravCompPort);

	// declaration of DH parameters, Mass vector and COG
	addProperty( "DOF", nrJoints ).doc("An unsigned integer that specifies the number of degrees of freedom");
	addProperty( "a", a ).doc("An Eigen::MatrixXd vector containing DH parameters a");
	addProperty( "d", d ).doc("An Eigen::MatrixXd vector containing DH parameters d");
	addProperty( "alpha", alpha ).doc("An Eigen::MatrixXd vector containing DH parameters alpha");
	addProperty( "theta", alpha ).doc("An Eigen::MatrixXd vector containing DH parameters theta");
	addProperty( "m", m ).doc("mass vector m stored in an Eigen::MatrixXd vector");
	
	cog.setZero(3,nrJoints);
	addProperty( "COGx", cog(0) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGy", cog(1) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
	addProperty( "COGz", cog(2) ).doc("center of gravity coordinate vector x stored in an Eigen::MatrixXd matrix");
}


GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
	Istore.setZero(3,3*nrJoints);
	grav << -9.81,0,0;
	
	return true;
}

bool GravityTorques::startHook()
{
	log(Warning)<<"ARM GravityTorques: StartHook complete "<<endlog();

	if ( !jointAnglesPort.connected() ){
		log(Error)<<"ARM GravityTorques: jointAnglesPort not connected!"<<endlog();
		return false;
	}
	if ( !gravCompPort.connected() ){
		log(Warning)<<"ARM GravityTorques: Outputport not connected!"<<endlog();
	}

	return true;
}

void GravityTorques::updateHook()
{
	doubles jointAngles(nrJoints,0.0);
	doubles gravityTorques(nrJoints,0.0);
	Eigen::MatrixXd q;
	Eigen::MatrixXd gravityTorquesVector;
	q.setZero(nrJoints,1);
	gravityTorquesVector.setZero(nrJoints,1);
	
	
	jointAnglesPort.read(jointAngles);
	
	for (uint i=0; i<nrJoints; i++) {
		q(i,0) = jointAngles[i];
	}
	
	gravityTorquesVector = ComputeGravity(a,d,alpha,cog,m,Istore,q,grav);
	
	for(uint i=0; i<nrJoints; i++) {
		gravityTorques[i] = gravityTorquesVector(i,0);
	}
	
	gravCompPort.write(gravityTorques);
}

// Matrix computing the rotation matrix from link i-1 to link i, using the DH-parameters
Eigen::Matrix3d GravityTorques::ComputeRotationMatrix(double d,double alpha,double q){
	Eigen::Matrix3d R;
	R = Eigen::Matrix3d::Zero(3,3);
	R(0,0) = cos(q);
	R(0,1) = -sin(q)*cos(alpha);
	R(0,2) = sin(q)*sin(alpha);
	R(1,0) = sin(q);
	R(1,1) = cos(q)*cos(alpha);
	R(1,2) = -cos(q)*sin(alpha);
	R(2,0) = 0;
	R(2,1) = sin(alpha);
	R(2,2) = cos(alpha);
	return R;
}

// Matrix computing the rotation matrix from Euler angles
Eigen::Matrix3d GravityTorques::eul2rot(double phi, double theta, double psi){
	Eigen::Matrix3d R;
	R = Eigen::Matrix3d::Zero(3,3);
	R(0,0) = cos(phi)*cos(theta)*cos(psi) - sin(phi)*sin(psi);
	R(0,1) = -cos(phi)*cos(theta)*sin(psi) - sin(phi)*cos(psi);
	R(0,2) = cos(phi)*sin(theta);
	R(1,0) = sin(phi)*cos(theta)*cos(psi) + cos(phi)*sin(psi);
	R(1,1) = -sin(phi)*cos(theta)*sin(psi) + cos(phi)*cos(psi);
	R(1,2) = sin(phi)*sin(theta);
	R(2,0) = -sin(theta)*cos(psi);
	R(2,1) = sin(theta)*sin(psi);
	R(2,2) = cos(theta);
	return R;	
}

// Compute Recursive Newton Euler (only for revolute joints!!)
Eigen::MatrixXd GravityTorques::rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd cog,Eigen::MatrixXd m,Eigen::MatrixXd Istore,Eigen::MatrixXd Q,Eigen::MatrixXd Qd,Eigen::MatrixXd Qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext){
	// Declare matrices, vectors and scalars
	Eigen::MatrixXd R(3,3);
	Eigen::Vector3d pstar;
	Eigen::Vector3d r;
	Eigen::Vector3d wd;
	Eigen::Vector3d w;
	Eigen::Vector3d Iiw;
	Eigen::Vector3d v;
	Eigen::Vector3d vd;
	Eigen::Vector3d vhat;
	Eigen::Vector3d z0;
	Eigen::Vector3d f;
	Eigen::Vector3d nn;
	Eigen::Vector3d Rtranpos_pstar;
	Eigen::Vector3d F;
	Eigen::Vector3d N;  
	Eigen::MatrixXd Ii(3,3);
	Eigen::Vector3d Fmi;   
	int n,np;
		  
	// Rotation axis
	z0 << 0,0,1;    
	// Determine number of links
	n = m.cols();      
	// Determine if matrix or vector has to be computed
	np = Q.rows();
	// Define size of tau
	Eigen::MatrixXd tau(np,n);
	tau = Eigen::MatrixXd::Zero(np,n);
	
	// Declare dimension matrices
	Eigen::MatrixXd Fm(3,n);
	Eigen::MatrixXd Nm(3,n);
	Eigen::MatrixXd Rs(3,3*n);
	Eigen::MatrixXd pstarm(3,n);
	Eigen::MatrixXd q(n,1);
	Eigen::MatrixXd qd(n,1);
	Eigen::MatrixXd qdd(n,1);
	Eigen::MatrixXd Qtransposed(Q.cols(),Q.rows());
	Eigen::MatrixXd Qdtransposed(Qd.cols(),Qd.rows());
	Eigen::MatrixXd Qddtransposed(Qdd.cols(),Qdd.rows());
	f = Eigen::Vector3d::Zero();
	nn = Eigen::Vector3d::Zero();
	Fmi = Eigen::Vector3d::Zero();
	vhat = Eigen::Vector3d::Zero();
   
	Qtransposed = Q.transpose();
	Qdtransposed = Qd.transpose();
	Qddtransposed = Qdd.transpose();
   
	int i;
	for(i=1; i< np+1; i++){
	   // Collect q, qd, qdd
	   q = Qtransposed.block(0,i-1,n,1);
	   qd = Qdtransposed.block(0,i-1,n,1);
	   qdd = Qddtransposed.block(0,i-1,n,1);  
  
	   w = Eigen::Vector3d::Zero();
	   Iiw = Eigen::Vector3d::Zero();
	   wd = Eigen::Vector3d::Zero();
	   v = Eigen::Vector3d::Zero();
	   vd = grav;
	   Fm = Eigen::MatrixXd::Zero(3,n);
	   Nm = Eigen::MatrixXd::Zero(3,n);
	   pstarm = Eigen::MatrixXd::Zero(3,n);
	   Rs = Eigen::MatrixXd::Zero(3,3*n); // Rotation matrices
	   
	   // Compute link rotation matrices
	   int j;
	   for(j=1; j<n+1; j++){
		 Rs.block(0,3*j-3,3,3) = ComputeRotationMatrix(d(j-1),alpha(j-1),q(j-1)); // Fill Rs with the computed rotation matrics
		 pstarm.block(0,j-1,3,1) << a(j-1), d(j-1)*sin(alpha(j-1)), d(j-1)*cos(alpha(j-1));
	   }

	   //The forward recursion
	   int jj;
	   for(jj=1; jj<n+1; jj++){
		   R = Rs.block(0,3*jj-3,3,3).transpose();
		   pstar = pstarm.block(0,jj-1,3,1);
		   r = cog.block(0,jj-1,3,1); 
		   // compute omegadot_i wrt base frame (eq 7.152)
		   wd = R*(wd + z0*qdd(jj-1) + w.cross(z0*qd(jj-1)));
		   // compute omega_i (eq 7.149 + eq 7.150 but in GC axis of rot. is constantly z0 because of DH)
		   w = R*(w + z0*qd(jj-1));
		   // note that the computation of alpha_i (eq 7.153) is missing because it only relies on qd and qdd.
		   // compute acceleration of link i (eq 7.159)
		   vd = wd.cross(pstar) + w.cross(w.cross(pstar)) + R*vd;
		   // compute the acceleration of the end of link i (eq 7.158)
		   vhat = wd.cross(r) + w.cross(w.cross(r)) + vd;
		   // compute m_{i}*a_{c,i} which is part of eq 7.146 for the backward recursion. Note that this
		   // includes the gravity component for all links because the first link is given vd = grav.
		   F = m(0,jj-1)*vhat;
		   Ii = Istore.block(0,3*jj-3,3,3);
		   Iiw = Ii*w;
		   // (eq 7.136)
		   N = Ii*wd + w.cross(Iiw);
		   Fm.block(0,jj-1,3,1) = F;
		   Nm.block(0,jj-1,3,1) = N;
	   }
   
	   //The backward recursion
	   f = Fext.block(0,0,3,1);
	   nn = Fext.block(3,0,3,1);       
	   int k;
	   for(k=n; k>0; k--){
		   pstar = pstarm.block(0,k-1,3,1);
		   if(k==n){
			   R = Eigen::MatrixXd::Identity(3,3);
		   }
		   else{
			   R   = Rs.block(0,3*k,3,3);
		   }
		   r = cog.block(0,k-1,3,1);
		   Rtranpos_pstar = R.transpose()*pstar;
		   Fmi = Fm.block(0,k-1,3,1);
		   nn = R*(nn + Rtranpos_pstar.cross(f)) + (pstar+r).cross(Fmi) + Nm.block(0,k-1,3,1);
		   // compute the sum of forces on each link (eq 7.146)
		   f = R*f + Fm.block(0,k-1,3,1);
		   R = Rs.block(0,3*k-3,3,3);
		   // compute the total link torque. In case of gravity compensation this will only be
		   // the result of the gravitational acceleration.
		   tau(i-1,k-1) = nn.dot(R.transpose()*z0);
	   }
	 }
		
	 return tau;	
}

Eigen::MatrixXd GravityTorques::ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd cog,Eigen::MatrixXd m,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav){
	// Declare matrices and vectors 
	Eigen::MatrixXd Gravity(1,m.cols());
	Eigen::MatrixXd no_Fext(6,1);
	
	// No external force to compute gravitational forces
	no_Fext = Eigen::MatrixXd::Zero(6,1);
	
	// Compute gravitational forces  
	Gravity = rne(a,d,alpha,cog,m,Istore,q.transpose(),0*q.transpose(),0*q.transpose(),grav,no_Fext);
	
	return Gravity.transpose();	
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)

