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
    addPort("out1",gravityTorquesPort1);
    addPort("out2",gravityTorquesPort2);
    addPort("out3",gravityTorquesPort3);

    // declaration of arm parameters and gravityvector
    addProperty( "nrJoints", nrJoints ).doc("An unsigned integer that specifies the number of degrees of freedom");
    addProperty( "joint_type", joint_type ).doc("");
    addProperty( "joint_axis", joint_axis ).doc("");
    addProperty( "rotation_axis", rotation_axis ).doc("");
    addProperty( "rotation_angle", rotation_angle ).doc("");
    addProperty( "translation_X", translation_X ).doc("");
    addProperty( "translation_Y", translation_Y ).doc("");
    addProperty( "translation_Z", translation_Z ).doc("");
	addProperty( "masses", masses ).doc("mass vector m stored in an Eigen::MatrixXd vector");
    addProperty( "COGx", COGx ).doc("center of gravity coordinate vector x");
    addProperty( "COGy", COGy ).doc("center of gravity coordinate vector y");
    addProperty( "COGz", COGz ).doc("center of gravity coordinate vector z");
    addProperty( "GravityVector", GravityVector ).doc("Gravity Vector depends on choice of base frame. (array(0.0, 0.0, -9.81) for negative z direction)");

}

GravityTorques::~GravityTorques(){}

bool GravityTorques::configureHook()
{
    if (nrJoints >= MAXJOINTS) {
        log(Error)<<"ARM GravityTorques: Could not configure Gravity torques component: The number of joints " << nrJoints << " exceeds the maximum number of joints " << MAXJOINTS << "!"<<endlog();
        return false;
    }

    //! Parameter input checks
    if ( joint_type.size() != nrJoints || joint_axis.size() != nrJoints || rotation_angle.size() != nrJoints || rotation_axis.size() != nrJoints || translation_X.size() != nrJoints|| translation_Y.size() != nrJoints|| translation_Z.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized arm parameters!"<<endlog();
        return false;
    }

    if ( masses.size() != nrJoints || COGx.size() != nrJoints || COGy.size() != nrJoints || COGz.size() != nrJoints ) {
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component due to Wrongly sized masses, COGx, COGy or COGz!"<<endlog();
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
    log(Warning)<<"ARM GravityTorques: Number of masses: [" << nrMasses  << "]" <<endlog();

    //! Construct chains and solvers for each mass
    for (uint i = 0; i < nrMasses; i++) {
        log(Warning)<<"ARM GravityTorques: Constructing Chain and Solver to link: [" << mass_indexes[i]  << "] and with mass: [" << masses[mass_indexes[i]] << "]" <<endlog();
        for (uint j = 0; j < (mass_indexes[i]+1); j++) {

            KDL::Segment Segment_;
            KDL::Frame Frame_;
            KDL::Vector Vector_;
            string rotation;

            // Determine Vector of translation
            if (j == mass_indexes[i]){ // (if last segment, then frame is positioned in COG)
                Vector_ = Vector(COGx[j],COGy[j],COGz[j]);
                rotation = "";

            }
            else { // (else frame is positioned in next joint)
                Vector_ = Vector(translation_X[j],translation_Y[j],translation_Z[j]);
            }

            // Determine Frame of fixed rotation and translation
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

            log(Warning)<<"ARM GravityTorques: Revolute joint around " << joint_axis[j] << ", " << rotation << rotation_angle[j] << ", Translate with [" << translation_X[j] << "," << translation_Y[j] << "," << translation_Z[j] << "]" <<endlog();

        }

        log(Warning)<<"ARM GravityTorques: Size of RobotArmChain is : " << RobotArmChain[i].segments.size() << "!"<<endlog();

        jacobian_solver[i] = new KDL::ChainJntToJacSolver(RobotArmChain[i]);
    }

    return true;
}

bool GravityTorques::startHook()
{
    //! Connection checks
    if ( !jointAnglesPort.connected() ){
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: jointAnglesPort not connected!"<<endlog();
        return false;
    }
    if ( !gravityTorquesPort1.connected() ){
        log(Error)<<"ARM GravityTorques: Could not start Gravity torques component: Outputport not connected!"<<endlog();
        return false;
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
        doubles gravityTorques1(nrJoints,0.0);
        gravityTorques1 = ComputeGravityTorques(q_current_,0);

        doubles gravityTorques2(nrJoints,0.0);
        gravityTorques2 = ComputeGravityTorques(q_current_,1);

        doubles gravityTorques3(nrJoints,0.0);
        gravityTorques3 = ComputeGravityTorques(q_current_,2);

        //write gravity torques
        gravityTorquesPort1.write(gravityTorques1);
        gravityTorquesPort2.write(gravityTorques2);
        gravityTorquesPort3.write(gravityTorques3);
    }
}

doubles GravityTorques::ComputeGravityTorques(KDL::JntArray q_current_, int i)
{
    doubles gravityTorques_(nrJoints,0.0);

    // Joint Torques are calculated by multiplying the wrench for each link by the corresponding partial Jacobian
    //for (uint i=0; i<nrMasses; i++) {

    //! Determine Jacobian (KDL)
    KDL::Jacobian partial_jacobian_KDL_(nrJoints);
    jacobian_solver[i]->JntToJac(q_current_, partial_jacobian_KDL_);

    //Convert Jacobian of type KDL to type Eigen::MatrixXd
    Eigen::MatrixXd partial_jacobian_(partial_jacobian_KDL_.rows(), partial_jacobian_KDL_.columns());
    for (uint jc=0; jc<partial_jacobian_KDL_.columns(); jc++) {
        for (uint jr=0; jr<partial_jacobian_KDL_.rows(); jr++) {
            partial_jacobian_(jr,jc) = partial_jacobian_KDL_(jr,jc);
        }
    }

    ///////////////////////////////



    //! Determine GravityWrench_
//    KDL::Frame Root_Frame = RobotArmChain[i].getSegment(0).getFrameToTip();
//    KDL::Frame Link_Frame = RobotArmChain[i].getSegment(mass_indexes[i]).getFrameToTip();
//    KDL::Twist TwistVec_;
//     TwistVec_ = KDL::diff(Root_Frame,Link_Frame);

//    KDL::Wrench GravityWrenchGlobal_ = KDL::Wrench::Zero();
 //   GravityWrenchGlobal_[2] = 9.81*masses[mass_indexes[i]];


  //  GravityWrenchKDL_ = GravityWrenchGlobal_.RefPoint(KDL::Vector(TwistVec_.vel.x(), TwistVec_.vel.y(), TwistVec_.vel.z()));


    //::Wrench GravityWrenchKDL_COG_ = GravityWrenchGlobal*masses[mass_indexes[i]];

    // calculate COG_translation vector (vector in tip frame)
    //KDL::Vector COG_translation_tipframe_;
    //COG_translation_tipframe_(0) = COGx[mass_indexes[i]];
    //COG_translation_tipframe_(1) = COGy[mass_indexes[i]];
    //COG_translation_tipframe_(2) = COGz[mass_indexes[i]];

    // transform gravityWrench in COG to centerpoint of the link frame (although still expressed relative to base frame)
    //KDL::Wrench GravityWrenchKDL_ = GravityWrenchKDL_COG_.RefPoint(COG_translation_tipframe_);
    //KDL::Wrench GravityWrenchKDL_ = KDL::Wrench::Zero();

    // Convert KDL GravityWrench to Eigen::Vector GravityWrench


    ///////////////////////////////////



    //! Determine GravityWrench_
    Eigen::VectorXd GravityWrench_ = Eigen::VectorXd::Zero(6);
    for (uint l=0; l<3; l++) {
        GravityWrench_(l) = GravityVector[l]*masses[mass_indexes[i]];
    }

    //! Calculate partial gravity torque
    Eigen::VectorXd PartialGravityTorques_ = Eigen::VectorXd::Zero(nrJoints);
    PartialGravityTorques_ = partial_jacobian_.transpose()*GravityWrench_;

    //! convert partial gravity torque to doubles and add all partial gravity torques together
    for (uint j=0; j<nrJoints; j++) {
        gravityTorques_[j] = PartialGravityTorques_(j);
    }
   // }

    return gravityTorques_;
}

ORO_CREATE_COMPONENT(ARM::GravityTorques)
