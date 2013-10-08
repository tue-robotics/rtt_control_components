#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <rtt/Component.hpp>
#include <tf/transform_broadcaster.h>

#include "PublishOdometry.hpp"

using namespace std;
using namespace RTT;
using namespace MSG;

PublishOdometry::PublishOdometry(const std::string& name) : TaskContext(name)
{
    // Creating ports
    addPort( "pos", pos_port );
    addPort( "odom", odom_port );

    addProperty( "base_link_frame", base_link_frame );
    addProperty( "odom_frame", odom_frame );

}

PublishOdometry::~PublishOdometry(){}

bool PublishOdometry::configureHook()
{
    return true;
}

bool PublishOdometry::startHook()
{
    old_time = os::TimeService::Instance()->getNSecs()*1e-9;
    prev_pos.assign(3,0.0);
    global_px = 0.0;
    global_py = 0.0;

    if (base_link_frame == "" || odom_frame == "") {
        log(Error)<<"Base link frame or odom frame not specified"<<endlog();
        return false;
    }
    return true;
}

void PublishOdometry::updateHook()
{
    ros::Time current_time = ros::Time::now(); // Change to wall-time?
    long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
    double dt = new_time - old_time;
    old_time = new_time;

    // Read position port
    doubles pos(3);
    pos_port.read(pos);

    // Compute velocities in robot frame
    doubles vel(3);
    for ( uint i = 0; i <= 2; i++ )
    {
        vel[i] = ( pos[i] - prev_pos[i] )/dt;
        prev_pos[i] = pos[i];
    }

    // Compote angle of movement
    double costh = cos(pos[2]);
    double sinth = sin(pos[2]);

    // Velocity in global frame
    double global_vx = (vel[0] * costh - vel[1] * sinth);
    double global_vy = (vel[0] * sinth + vel[1] * costh);

    // Resulting position delta in global frame
    double delta_x = global_vx * dt;
    double delta_y = global_vy * dt;

    // Global position
    global_px = global_px + delta_x;
    global_py = global_py + delta_y;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos[2]);

    // Instantiate odometry message and odometry tf message
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_tf;

    // Populate odom msg
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_link_frame;
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = global_px;
    odom.pose.pose.position.y = global_py;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vel[0];
    odom.twist.twist.linear.y = vel[1];
    odom.twist.twist.angular.z = vel[2];

    // Populate tf msg
    odom_tf.header.frame_id = odom_frame;
    odom_tf.child_frame_id = base_link_frame;
    odom_tf.header.stamp = current_time;
    odom_tf.transform.translation.x = global_px;
    odom_tf.transform.translation.y = global_py;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom_quat;

    // An odometry message has a 6x6 covariance matrix
    odom.pose.covariance.assign(0.0);
    double sigma_x, sigma_y, sigma_phi, cov_x_y, cov_x_phi, cov_y_phi;
    if(fabs(vel[0]) <= 1e-8 && fabs(vel[1]) <= 1e-8 && fabs(vel[2]) <= 1e-8)
    {
        sigma_x = 1e-6; //WHY?
        sigma_y = 1e-6; //WHY?
        sigma_phi = 1e-6; //WHY?
        cov_x_y = 1e-12;
        cov_x_phi = 1e-12;
        cov_y_phi = 1e-12;
    }
    else
    {
        sigma_x = 0.002; //WHY?
        sigma_y = 0.002; //WHY?
        sigma_phi = 0.017; //WHY?
        cov_x_y = 0.0;
        cov_x_phi = 0.0;
        cov_y_phi = 0.0;
    }
    odom.pose.covariance[0] = pow(sigma_x,2);
    odom.pose.covariance[7] = pow(sigma_y,2);
    odom.pose.covariance[35] = pow(sigma_phi,2);

    odom.pose.covariance[1] = cov_x_y;
    odom.pose.covariance[6] = cov_x_y;
    odom.pose.covariance[31] = cov_y_phi;
    odom.pose.covariance[11] = cov_y_phi;
    odom.pose.covariance[30] = cov_x_phi;
    odom.pose.covariance[5] = cov_x_phi;

    odom.pose.covariance[14] = DBL_MAX;
    odom.pose.covariance[21] = DBL_MAX;
    odom.pose.covariance[28] = DBL_MAX;

    odom.twist.covariance = odom.pose.covariance;

    // Publish the odom message
    odom_port.write(odom);
    // Publish the tf message
    tf_broadcaster.sendTransform(odom_tf);
}


ORO_CREATE_COMPONENT(MSG::PublishOdometry)
