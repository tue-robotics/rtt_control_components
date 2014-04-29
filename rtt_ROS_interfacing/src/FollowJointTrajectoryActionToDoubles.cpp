//#include <rtt/TaskContext.hpp>
//#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "FollowJointTrajectoryActionToDoubles.hpp"
#define DT 0.001

using namespace std;
using namespace RTT;
using namespace ROS;

FollowJointTrajectoryActionToDoubles::FollowJointTrajectoryActionToDoubles(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "NumberOfJoints", Ndouble_ );
    addProperty( "max_dx", max_dx );
    addPort( "in", goalport );
    addPort( "pos_out", position_outport_ );
    addPort( "eff_out", effort_outport_ );
}

FollowJointTrajectoryActionToDoubles::~FollowJointTrajectoryActionToDoubles(){}

bool FollowJointTrajectoryActionToDoubles::configureHook()
{
    eff_out_.assign(Ndouble_, 0.0);
    pos_out_.assign(Ndouble_, 0.0);
    last_pos_out_.assign(Ndouble_, 0.0);
    joint_states.assign(Ndouble_, 0.0);
    pos.assign(Ndouble_, 0.0);
    return true;
}

bool FollowJointTrajectoryActionToDoubles::startHook()
{
    /// Check which ports are connected
    if (!position_outport_.connected())
    {
        log(Warning)<<"ReadJointState: Position outport not connected"<<endlog();
    }
    if (!effort_outport_.connected())
    {
        log(Info)<<"ReadJointState: Effort outport not connected"<<endlog();
    }
    if (!goalport.connected())
    {
        log(Warning)<<"ReadJointState: Inport not connected"<<endlog();
    }
    playing_trajectory = false;

    return true;
}

void FollowJointTrajectoryActionToDoubles::updateHook()
{

    if ( goalport.read(goalmsg) == NewData) 
    {
		// TODO: If the emergency button is pressed, a continuous stream of new goals containing current measured positions should be send
		tp = 0;
        playing_trajectory = true;
        playing_trajectory_point = false;
	}
	
	if ( playing_trajectory && ! playing_trajectory_point ) // Lets calculate the speeds to the next point
	{
		double max_vels[8] = {}; //TODO: What is the current arm position?
		double max_accs[8] = {}; //TODO: What is the current arm position?
		double times[8] = {}; //TODO: What is the current arm position?
		
		// find out how long it will take to reach the point
        // taking into account acceleration and deceleration
        goal_pos = goalmsg.goal.trajectory.points[tp].positions;
        
        double max_time = 0;
        for(unsigned int j = 0; j < goal_pos.size(); ++j) {           
			double state = joint_states[j];
			double max_vel = max_vels[j];
			double max_acc = max_accs[j];
			
            double diff = std::abs(goal_pos[j] - state);

            double t_acc = max_vel / max_acc;
            double x_acc = max_acc * t_acc * t_acc / 2;

            double time;
            if (x_acc < diff / 2) {
                // reach full velocity
                double x_max_vel = diff - (2 * x_acc);
                time = x_max_vel / max_vel + 2 * t_acc;
            } else {
                // do not reach full velocity
                double t_half = sqrt(2 * (diff / 2) / max_acc);
                time = 2 * t_half;
            }

            times[j] = time;

            max_time = std::max(max_time, time);
        }
                    
        // scale the maximum velocity and acceleration of each joint based on the longest time
        for(unsigned int j = 0; j < joint_states.size(); ++j) {
            double time_factor = times[j] / max_time;
            cur_max_acc[j] = max_accs[j] * time_factor * time_factor;
            cur_max_vel[j] = max_vels[j] * time_factor;
        }
        
        playing_trajectory_point = true;
        tp++;
	}
	
	if (true) //(Criterium goal reached)
	{
		playing_trajectory_point = false;
		if (tp >= goalmsg.goal.trajectory.points.size())
		{
			// Send action result
			control_msgs::FollowJointTrajectoryActionResult resultmsg;
			resultmsg.result.error_code = 0; // SUCCESSFUL
			resultmsg.status.status = 3;     // SUCCEEDED
			
			playing_trajectory = false;
		}
	}

	if (playing_trajectory && playing_trajectory_point)
	{
		for(unsigned int j = 0; j < joint_states.size(); ++j) {
			

			double v_sign;
			double v_abs = abs(cur_max_vel[j], v_sign);

			// calculate distance to goal
			double dx = goal_pos[j] - pos[j];
			double dx_sign;
			double dx_abs = abs(dx, dx_sign);

			// calculate deceleration time and distance
			double dt_dec = v_abs / cur_max_acc[j];
			double dx_dec = 0.5 * cur_max_acc[j] * dt_dec * dt_dec;

			if (dx_abs <= dx_dec) {
				// decelerate
				v_abs = std::max(0.0, v_abs - cur_max_acc[j] * DT);
			} else {
				// accelerate
				v_abs = std::min(cur_max_vel[j], v_abs + cur_max_acc[j] * DT);
			}

			double vel = dx_sign * v_abs;
			pos[j] += vel * DT;

			pos_out_[j] = pos[j];

		}
		position_outport_.write(pos_out_);
    }
}

ORO_CREATE_COMPONENT(ROS::FollowJointTrajectoryActionToDoubles)
