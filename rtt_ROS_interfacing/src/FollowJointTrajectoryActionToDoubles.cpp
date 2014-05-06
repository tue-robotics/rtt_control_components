//#include <rtt/TaskContext.hpp>
//#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include "FollowJointTrajectoryActionToDoubles.hpp"
#define DT 0.001
#define EPS 0.001

using namespace std;
using namespace RTT;
using namespace ROS;

FollowJointTrajectoryActionToDoubles::FollowJointTrajectoryActionToDoubles(const string& name) :
    TaskContext(name, PreOperational)
{
    addProperty( "NumberOfJoints", Nj );
    addProperty( "start_pos", start_pos );
    addProperty( "max_vels", max_vels );
    addProperty( "max_accs", max_accs );
    addPort( "goal", goalport );
    addPort( "pos_out", position_outport_ );
    addPort( "resetValues", resetPort );
    addPort( "result", resultport );
}

FollowJointTrajectoryActionToDoubles::~FollowJointTrajectoryActionToDoubles(){}

bool FollowJointTrajectoryActionToDoubles::configureHook()
{
    pos.assign(Nj, 0.0);
    goal_pos.assign(Nj, 0.0);
    cur_max_vel.assign(Nj, 0.0);
    cur_max_acc.assign(Nj, 0.0);
    return true;
}

bool FollowJointTrajectoryActionToDoubles::startHook()
{
    /// Check which ports are connected
    if (!position_outport_.connected())
    {
        log(Warning)<<"ReadJointState: Position outport not connected"<<endlog();
    }
    if (!goalport.connected())
    {
        log(Warning)<<"ReadJointState: Inport not connected"<<endlog();
    }
            
    for(unsigned int j = 0; j < Nj; ++j) {
		pos[j] = start_pos[j];
	}

    playing_trajectory = false;
    return true;
}

void FollowJointTrajectoryActionToDoubles::updateHook()
{
	doubles resetdata;
	if (resetPort.read( resetdata ) == NewData) // Following the trajectory is interupted and the actual joints may be moved
	{
		for(unsigned int j = 0; j < Nj; ++j) 
		{
			pos[j] = resetdata[j];
		}
		playing_trajectory = false;        
		return;
	}

    if ( goalport.read(goalmsg) == NewData) 
    {
		tp = 0;
        playing_trajectory = true;
        playing_trajectory_point = false;
		log(Warning)<<"New goal received"<<endlog();
	}
	
	if ( playing_trajectory && ! playing_trajectory_point ) // Lets calculate the speeds to the next point
	{
		doubles durations; //How long does the movement take per joint
		durations.assign(Nj, 0.0);
		
		// find out how long it will take to reach the point
        // taking into account acceleration and deceleration
        goal_pos = goalmsg.goal.trajectory.points[tp].positions;
        
        //TODO: Check goal_pos with Ndouble
         
        double max_duration = 0.0;
        for(unsigned int j = 0; j < Nj; ++j) {           
			double max_vel = max_vels[j];
			double max_acc = max_accs[j];
			
            double diff = std::abs(goal_pos[j] - pos[j]);

            double t_acc = max_vel / max_acc;
            double x_acc = max_acc * t_acc * t_acc / 2;

            double duration;
            if (x_acc < diff / 2) {
                // reach full velocity
                double x_max_vel = diff - (2 * x_acc);
                duration = x_max_vel / max_vel + 2 * t_acc;
            } else {
                // do not reach full velocity
                double t_half = sqrt(2 * (diff / 2) / max_acc);
                duration = 2 * t_half;
            }

            durations[j] = duration;

            max_duration = std::max(max_duration, duration);
        }
             
        // scale the maximum velocity and acceleration of each joint based on the longest duration
        for(unsigned int j = 0; j < Nj; ++j) {
            double time_factor = durations[j] / max_duration;
            cur_max_acc[j] = max_accs[j] * time_factor * time_factor;
            cur_max_vel[j] = max_vels[j] * time_factor;
        }
         
        playing_trajectory_point = true; // We calculated the vels and accs per joint, lets play
        tp++; // Next time this function should use the next point in the trajectory message
		
		log(Warning)<<"New trajectory to next point calculated"<<endlog();
	}
	
	if ( playing_trajectory && playing_trajectory_point && std::abs(goal_pos[0] - pos[0]) < EPS ) //(Criterium goal reached)
	{
		// This implementation does not work, what if the first joint does not move?
		playing_trajectory_point = false;
		log(Warning)<<"Trajectory point reached"<<endlog();
		if (tp >= goalmsg.goal.trajectory.points.size())
		{
			// Send action result
			control_msgs::FollowJointTrajectoryActionResult resultmsg;
			resultmsg.result.error_code = 0; // SUCCESSFUL
			resultmsg.status.status = 3;     // SUCCEEDED
			resultport.write(resultmsg);
			playing_trajectory = false;
			log(Warning)<<"Trajectory finished"<<endlog();

		}
	}

	if (playing_trajectory && playing_trajectory_point)
	{
		for(unsigned int j = 0; j < Nj; ++j) {
			
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


		}
		log(Warning)<<"Going to: " << pos[0] <<", " << pos[1] <<", " << pos[2] <<", " << pos[3] <<", " << pos[4] <<", " << pos[5] <<", " << pos[6] <<endlog();

		position_outport_.write(pos);
    }
}

ORO_CREATE_COMPONENT(ROS::FollowJointTrajectoryActionToDoubles)
