#include "TrajectoryActionlib.hpp"

#include <ros/console.h>

using namespace std;
using namespace RTT;
using namespace ROS;

template <typename T>
std::string vectorToString(std::vector<T> vector)
{
  std::stringstream ss;
  for (const T& e : vector)
  {
    ss << e << ", ";
  }
  return ss.str();
}

TrajectoryActionlib::TrajectoryActionlib(const string& name) : TaskContext(name, PreOperational)
{
  ROS_DEBUG_NAMED("TrajectoryActionlib", "Constructor");

	//! Operations
	addOperation("AddBodyPart", &TrajectoryActionlib::AddBodyPart, this, OwnThread)
		.doc("Add a body part by specifying its partNr and its jointNames")
		.arg("partNr","The number of the bodypart")
		.arg("JointNames","The name of joints");
	addOperation( "SendToPos", &TrajectoryActionlib::SendToPos, this, OwnThread )
		.doc("Send the bodypart to position, used when finished homing")
		.arg("partNr","The number of the bodypart")     
		.arg("pos","Position to go to"); 
	addOperation( "ResetReferences", &TrajectoryActionlib::ResetReferences, this, OwnThread )
		.doc("Reset the reference generator to measured current position (used in homing)")
		.arg("partNr","The number of the bodypart");
	
	//! Actionlib
	// Add action server ports to this task's root service
	rtt_action_server_.addPorts(this->provides());

	// Bind action server goal and cancel callbacks
	rtt_action_server_.registerGoalCallback(boost::bind(&TrajectoryActionlib::goalCallback, this, _1));
	rtt_action_server_.registerCancelCallback(boost::bind(&TrajectoryActionlib::cancelCallback, this, _1));

	//! AddAttribute
	addAttribute( "allowedBodyparts", allowedBodyparts );
}

TrajectoryActionlib::~TrajectoryActionlib()
{
  ROS_DEBUG_NAMED("TrajectoryActionlib", "Destructor");

	//! remove operations
	remove("AddBodyPart");
	remove("SendToPos");
	remove("ResetReferences");
}

bool TrajectoryActionlib::configureHook()
{
  ROS_DEBUG_NAMED("TrajectoryActionlib", "configureHook");

	//! Resize
	minpos.resize(maxN);
	maxpos.resize(maxN);
	maxvel.resize(maxN);
	maxacc.resize(maxN);
	actualPos.resize(maxN);
	desiredPos.resize(maxN);
	desiredVel.resize(maxN);
	desiredAcc.resize(maxN);
	pos_out.resize(maxN);
	vel_out.resize(maxN);
	acc_out.resize(maxN);
	allowedBodyparts.resize(maxN);
	allowedBodyparts_prev.resize(maxN);

	//! Init
	checked = false;
	totalNumberOfJoints = 0;
	numberOfBodyparts = 0;
	start_time = 0.0;
	vector_sizes.assign(maxN,0);
	InterpolDts.assign(maxN,0.0);
	InterpolEpses.assign(maxN,0.0);

	return true;
}

bool TrajectoryActionlib::startHook()
{
  ROS_DEBUG_NAMED("TrajectoryActionlib", "StartHook");

	// Initialize
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;

	// Start the actionlib server
	rtt_action_server_.start();
	dt = this->getPeriod();
	dt = 0.001;
	return true;
}

void TrajectoryActionlib::updateHook()
{
  ROS_DEBUG_THROTTLE_NAMED(1., "TrajectoryActionlib", "updateHook 1 Hz throttled");

	double t_now = os::TimeService::Instance()->getNSecs()*1e-9;
	// 6.5s after start, check all properties, ports, etc.
	if (!checked) {
		double aquisition_time = t_now;
		if ( aquisition_time - start_time > 6.5) {
			if (!CheckConnectionsAndProperties()) {
				this->stop();
			} else {
				checked = true;
			}
		} else {
			return;
		}
	}
	
	// If a goal is active
	if (reference_generator_.hasActiveGoals())
	{
		//log(Info) << "TrajectoryActionlib: Goal active!" << endlog();
		const std::vector<std::string>& joint_names = reference_generator_.joint_names();

		std::vector<double> current_positions(joint_names.size(), 0);
		for(unsigned int i = 0 ; i < joint_names.size(); ++i) {
			map<string, BodyJointPair>::const_iterator it = joint_map.find(joint_names[i]);
			BodyJointPair bjp = it->second;
			int body_part_id = bjp.first;
			int joint_id = bjp.second;
			current_positions[i] = desiredPos [body_part_id] [joint_id];
		}

		std::vector<double> references;
    if (reference_generator_.calculatePositionReferences(dt, references))
    {
      for(unsigned int i = 0 ; i < joint_names.size(); ++i) {
        map<string, BodyJointPair>::const_iterator it = joint_map.find(joint_names[i]);
        BodyJointPair bjp = it->second;
        int body_part_id = bjp.first;
        int joint_id = bjp.second;

        // Joint positions
        if (references[i] == references[i]) // Check for NaN
        {
          desiredPos[body_part_id][joint_id] = references[i];
        }
        actualPos[body_part_id] [joint_id] = desiredPos[body_part_id][joint_id];

        // Joint velocities
        double vel = reference_generator_.joint_state(i).velocity();
        if (vel == vel) // Check for NaN
        {
          vel_out[body_part_id][joint_id] = vel;
        }

        // Joint accelerations
        double acc = reference_generator_.joint_state(i).acceleration();
        if (acc == acc) // Check for NaN
        {
          acc_out[body_part_id][joint_id] = acc;
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to calculatePositionReferences, aborting all active goals");
      reference_generator_.abortAllGoals();
    }
		
		// Check for each goal if it succeeded or canceled, and notify the goal handle accordingly
		for(std::map<std::string, GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
		{
			GoalHandle& gh = it->second;
			tue::manipulation::JointGoalStatus status = reference_generator_.getGoalStatus(gh.getGoalID().id);

			if (status == tue::manipulation::JOINT_GOAL_SUCCEEDED)
			{
				gh.setSucceeded();
				goal_handles_.erase(it++);
			}
			else if (status == tue::manipulation::JOINT_GOAL_CANCELED)
			{
				gh.setCanceled();
				goal_handles_.erase(it++);
			}
      else if (status == tue::manipulation::JOINT_GOAL_ABORTED)
      {
        gh.setAborted();
        goal_handles_.erase(it++);
      }
			else
			{
				++it;
			}
		}

		for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
			uint partNr = activeBodyparts[j];
			if (allowedBodyparts[partNr-1] == true) {
				// if (partNr == 2)
				// {
				// 	log(Info) << "TAL: torso ref: " << desiredPos[partNr-1][0] << ", vel: " << vel_out[partNr-1][0] << ", acc: " << acc_out[partNr-1][0] << endlog();
				// }
				posoutport[partNr-1].write( desiredPos[partNr-1] );
				veloutport[partNr-1].write( vel_out[partNr-1] );
				accoutport[partNr-1].write( acc_out[partNr-1] );
			}
		}
	}
}

// Called by rtt_action_server_ when a new goal is received
void TrajectoryActionlib::goalCallback(GoalHandle gh) {
	log(Info) << "TrajectoryActionlib: Received Message" << endlog();
  ROS_INFO_STREAM_NAMED("TrajectoryActionlib", "goalCallback goal: " << *gh.getGoal());

	std::stringstream error;
  std::string goalid = gh.getGoalID().id;
	if (!reference_generator_.setGoal(*gh.getGoal(), goalid, error))
	{
		gh.setRejected();
		ROS_ERROR("%s", error.str().c_str());
		return;
	}

	// Accept the goal
	gh.setAccepted();
  goal_handles_[goalid] = gh;
}

// Called by rtt_action_server_ when a goal is cancelled / preempted
void TrajectoryActionlib::cancelCallback(GoalHandle gh)
{
  ROS_INFO_STREAM_NAMED("TrajectoryActionlib", "cancelCallback goal: " << *gh.getGoal());

	gh.setCanceled();
	reference_generator_.cancelGoal(gh.getGoalID().id);
	goal_handles_.erase(gh.getGoalID().id);
}

void TrajectoryActionlib::AddBodyPart(int partNr, strings JointNames)
{
  ROS_DEBUG_STREAM_NAMED("TrajectoryActionlib", "AddBodyPart: " << partNr << vectorToString(JointNames));

	// Update map
	for (uint l = 0; l < JointNames.size(); l++) {
		joint_map[JointNames[l]] = make_pair(partNr-1, l);
	}

	// Initialize various vectors of vectors
	pos_out[partNr-1].assign(JointNames.size(),0.0);
	vel_out[partNr-1].assign(JointNames.size(),0.0);
	acc_out[partNr-1].assign(JointNames.size(),0.0);
	actualPos[partNr-1].assign(JointNames.size(),0.0);
	desiredPos[partNr-1].assign(JointNames.size(),0.0);
	desiredVel[partNr-1].assign(JointNames.size(),0.0);
	desiredAcc[partNr-1].assign(JointNames.size(),0.0);
	minpos[partNr-1].assign(JointNames.size(),0.0);
	maxpos[partNr-1].assign(JointNames.size(),0.0);
	maxvel[partNr-1].assign(JointNames.size(),0.0);
	maxacc[partNr-1].assign(JointNames.size(),0.0);

	// Add ports
	addPort(        ("pos_out"+to_string(partNr)),          posoutport[partNr-1] )      .doc("Position Reference");
	addPort(        ("vel_out"+to_string(partNr)),          veloutport[partNr-1] )      .doc("Velocity Reference");
	addPort(        ("acc_out"+to_string(partNr)),          accoutport[partNr-1] )      .doc("Acceleration Reference");
	addPort(        ("current_pos"+to_string(partNr)),      currentpos_inport[partNr-1]).doc("Inport for current position for initial position and feedtrough for disabled mode");

	// Add Properties
	addProperty(    "interpolatorDt"+to_string(partNr),     InterpolDts[partNr-1] )     .doc("Interpol Dt");
	addProperty(    "interpolatorEps"+to_string(partNr),    InterpolEpses[partNr-1] )   .doc("Interpol Eps");

	// Update total numer of joints and present bodyparts
	totalNumberOfJoints += JointNames.size();
	vector_sizes[partNr-1] = JointNames.size();
	numberOfBodyparts += 1;
	activeBodyparts.resize(numberOfBodyparts);
	activeBodyparts[numberOfBodyparts-1] = partNr;
	allowedBodyparts[partNr-1] = false;

	log(Info) << "TrajectoryActionlib: Total of "<< totalNumberOfJoints <<" joints for " << numberOfBodyparts << " Bodyparts" << endlog();

	// Get the constraints for each joint from the URDF model
	urdf::Model Model;
	Model.initParam("robot_description");

	for (size_t i = 0; i < JointNames.size(); ++i)
	{
		urdf::JointConstSharedPtr Joint = Model.getJoint(JointNames[i]);
		if (Joint) 
		{
			minpos[partNr-1][i] = Joint->limits->lower; ///TODO/// Maybe not store in a matrix, are they used anywhere outside this loop?
			maxpos[partNr-1][i] = Joint->limits->upper;
			maxvel[partNr-1][i] = Joint->limits->velocity;
			maxacc[partNr-1][i] = Joint->limits->effort;

			log(Info) << "TrajectoryActionlib: Bodypart " << partNr << ", Joint " << JointNames[i] << " has these limits: "<< endlog();
			log(Info) << "minpos="<<minpos[partNr-1][i]<<" maxpos="<<maxpos[partNr-1][i]<<" maxvel="<<maxvel[partNr-1][i]<<" maxacc="<<maxacc[partNr-1][i]<<endlog();

			reference_generator_.initJoint(JointNames[i], Joint->limits->velocity, Joint->limits->effort, Joint->limits->lower, Joint->limits->upper);
			//reference_generator_.setPositionLimits(i, minpos[partNr-1][i], maxpos[partNr-1][i]);
		}
		else
		{
			log(Error) << "Joint Null Pointer for Model.getJoint() on '" << JointNames[i] << "'" << endlog();
		}
	}
}

void TrajectoryActionlib::SendToPos(int partNr, strings jointnames, doubles pos)
{
  ROS_DEBUG_STREAM_NAMED("TrajectoryActionlib", "SendToPos: " << partNr << "," << vectorToString(jointnames)
                        << ", " << vectorToString(pos));

  if (pos.size() != vector_sizes[partNr-1])
  {
		log(Warning) << "TrajectoryActionlib: Invalid size of pos/vector_sizes[partNr-1]" << endlog();
	}
  if (allowedBodyparts[partNr-1] == false)
  {
		log(Warning) << "TrajectoryActionlib: Received SendToPos for bodypart that is not yet allowed" << endlog();
	}
	log(Info)<< "TrajectoryActionlib: Received SendToPos goal: " << endlog();

	tue::manipulation::JointGoalInfo goal_info;
	bool result = reference_generator_.setGoal(jointnames, pos, goal_info);

	log(Info) << "TrajectoryActionlib::SendToPos: result = " << result << endlog();
	log(Info) << "Lets go!" << endlog();
}

void TrajectoryActionlib::ResetReferences(int partNr)
{
  ROS_DEBUG_STREAM_NAMED("TrajectoryActionlib", "ResetReferences: " << partNr);
	if (partNr < 0 || partNr > maxN ) {
		log(Error) <<"TrajectoryActionlib::ResetReferences: Invalid partNr provided: partNr = " << partNr <<endlog();
		return;
	}
	
	doubles resetpos;
	strings jointnames;
	ints jointids;
	//Set the starting value to the current actual value
	currentpos_inport[partNr-1].read( resetpos );

	for ( map<string, BodyJointPair>::const_iterator it = joint_map.begin(); it != joint_map.end(); ++it )
	{
		const BodyJointPair& body_joint_pair = it->second;
		if (body_joint_pair.first == partNr-1) 
		{
			jointnames.push_back(it->first);
			jointids.push_back(it->second.second);
		}
	}

	for (uint i = 0; i < jointnames.size(); ++i)
	{
		reference_generator_.resetJointState(jointnames[i], resetpos[jointids[i]]);
	}
	
	// Now the reference_generator_ has been resetted, also once this has to be written to the outputports
	doubles zeros;
	zeros.assign(resetpos.size(),0.0);
	posoutport[partNr-1].write( resetpos );
	veloutport[partNr-1].write( zeros );
	accoutport[partNr-1].write( zeros );

	return;
}

bool TrajectoryActionlib::CheckConnectionsAndProperties()
{
  ROS_DEBUG_THROTTLE_NAMED(1., "CheckConnectionsAndProperties", "updateHook 1 Hz throttled");

	// to do use iterator
	for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
		// look up bodypart number
		uint partNr = activeBodyparts[j];
		// Property checks
		if ( (minpos[partNr-1].size() != vector_sizes[partNr-1]) || (maxpos[partNr-1].size() != vector_sizes[partNr-1]) || (maxvel[partNr-1].size() != vector_sizes[partNr-1]) || (maxacc[partNr-1].size() != vector_sizes[partNr-1]) ) {
			log(Error)<<"TrajectoryActionlib: Stopping component: Sizes of bodypart "<<partNr<<" are wrong: minpos("<< minpos.size() <<"), maxpos("<< maxpos.size() <<"), maxvel("<< maxvel.size() <<"), maxacc("<< maxacc.size() <<") -> Should be size " << vector_sizes[partNr-1] <<"."<<endlog();
			return false;
		}
		for ( uint i = 0; i < vector_sizes[partNr-1]; i++ ){
			if ( minpos[partNr-1][i] == 0.0 && maxpos[partNr-1][i] == 0.0 ) {
				log(Warning)<<"TrajectoryActionlib: minPos and maxPos both specified 0.0. Thus maxPos and minPos boundaries are not taken into account"<<endlog();
			} else if ( minpos[partNr-1][i] > maxpos[partNr-1][i]) {
				log(Error)<<"TrajectoryActionlib: Stopping component: minPosition should be specified smaller than maxPosition"<<endlog();
				return false;
			}
			if ( ( maxvel[partNr-1][i] < 0.0) || ( maxacc[partNr-1][i] < 0.0) ) {
				log(Error)<<"TrajectoryActionlib: Stopping component: maxVelocity and maxAcceleration should be specified positive"<<endlog();
				return false;
			}
		}
		if (InterpolDts[partNr-1] <= 0.0) {
			log(Error)<<"TrajectoryActionlib: Stopping component: interpolDt should be larger than zero for bodypart: "<< partNr-1 << "."<<endlog();
			return false;
		}
		if (InterpolEpses[partNr-1] <= 0.0) {
			log(Error)<<"TrajectoryActionlib: Stopping component: InterpolEps should be larger than zero for bodypart: "<< partNr-1 << "."<<endlog();
			return false;
		}

		// Port Connection checks
		if ( !posoutport[partNr-1].connected() && !veloutport[partNr-1].connected() && !accoutport[partNr-1].connected() ) {
			log(Error)<<"TrajectoryActionlib: Stopping component: None of the ports: posout" << partNr << ", velout" << partNr << ", accout" << partNr << " is connected."<<endlog();
			return false;
		}
		if ( !currentpos_inport[partNr-1].connected() ) {
			log(Error)<<"TrajectoryActionlib: Stopping component: initial_pos" << partNr << " is not connected."<<endlog();
			return false;
		}
	}

	return true;
}


ORO_CREATE_COMPONENT(ROS::TrajectoryActionlib)
