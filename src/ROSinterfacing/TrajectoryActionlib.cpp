#include "TrajectoryActionlib.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

TrajectoryActionlib::TrajectoryActionlib(const string& name) : TaskContext(name, PreOperational)
{
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
    //! remove operations
    remove("AddBodyPart");
    remove("SendToPos");
    remove("ResetReferences");
}

bool TrajectoryActionlib::configureHook()
{
    //! Resize
    minpos.resize(maxN);
    maxpos.resize(maxN);
    maxvel.resize(maxN);
    maxacc.resize(maxN);
    desiredPos.resize(maxN);
    desiredVel.resize(maxN);
    desiredAcc.resize(maxN);
    pos_out.resize(maxN);
    vel_out.resize(maxN);
    acc_out.resize(maxN);
    current_position.resize(maxN);
    mRefGenerators.resize(maxN);
    mRefPoints.resize(maxN);
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
    // Initialize
    start_time = os::TimeService::Instance()->getNSecs()*1e-9;

    // Start the actionlib server
    rtt_action_server_.start();
    return true;
}

void TrajectoryActionlib::updateHook()
{
	if (allowedBodyparts != allowedBodyparts_prev) {
		log(Warning) << "TrajectoryActionlib:  Allowed:     [" << allowedBodyparts[0] << "," << allowedBodyparts[1] << "," << allowedBodyparts[2] << "," << allowedBodyparts[3] << "," << allowedBodyparts[4] << "]" <<endlog();
	}
	allowedBodyparts_prev = allowedBodyparts;
	
    // 6.5s after start, check all properties, ports, etc.
    if (!checked) {
        double aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
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
    
    // Read all current positions
    for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
        uint partNr = activeBodyparts[j];
        currentpos_inport[partNr-1].read( current_position[partNr-1] );
	}

    // If a goal is active
    if (!goal_handles_.empty())
    {
        // Take the first item in the queue
        TrajectoryInfo& t_info = goal_handles_.front();
        GoalHandle& gh = t_info.goal_handle;

        // If first point, set t_start from trajectory
        if (t_info.t_start == -1)
            t_info.t_start = os::TimeService::Instance()->getNSecs()*1e-9;

        // Take first point in the queue
        const trajectory_msgs::JointTrajectoryPoint& point = t_info.points.front();

        // Check whether we have to start with the point
        if (point.time_from_start.toSec() <= os::TimeService::Instance()->getNSecs()*1e-9 - t_info.t_start)
        {
            // Send point to 'controller'
            const std::vector<std::string>& joint_names = gh.getGoal()->trajectory.joint_names;

            // then loop over all joints within the message received
            uint k = 0;
            while (k < joint_names.size()) {
                map<string, BodyJointPair>::const_iterator it = joint_map.find(joint_names[k]);

                // Update the output and go to the next message.
                BodyJointPair bjp = it->second;
                int body_part_id = bjp.first;
                int joint_id = bjp.second;
                if (allowedBodyparts[body_part_id] == true) {
                    desiredPos [body_part_id] [joint_id] = point.positions[k];
                    desiredVel [body_part_id] [joint_id] = maxvel [body_part_id] [joint_id];
                    desiredAcc [body_part_id] [joint_id] = maxacc [body_part_id] [joint_id];
                } else { // Message received for bodypart that did not get the AllowReadReference!
                    log(Warning) << "TrajectoryActionlib: Message received for bodypart that did not get the AllowReadReference!" << endlog();
                    desiredPos[body_part_id][joint_id] = current_position[body_part_id][joint_id];
                    gh.setAborted();
                    goal_handles_.erase(goal_handles_.begin());
                }
                k++;
            }

            // Check if we are already there
            bool already_there = true;
            k = 0;
            while (k < joint_names.size()) {
                map<string, BodyJointPair>::const_iterator it = joint_map.find(joint_names[k]);

                // Update the output and go to the next message.
                BodyJointPair bjp = it->second;
                int body_part_id = bjp.first;
                int joint_id = bjp.second;

                // Remove point if we are within 0.1 error
                if ( abs( desiredPos[body_part_id][joint_id] - mRefPoints[body_part_id][joint_id].pos) > 0.2 )
                {
                    //log(Info) << "TrajectoryActionlib: Joint name: " << joint_names[k] << " error is too large: " << abs( desiredPos[body_part_id][joint_id] - mRefPoints[body_part_id][joint_id].pos) << endlog();
                    already_there = false;
                }

                k++;
            }

            // Pop point if we are there :)
            if (already_there)
            {
                t_info.points.pop();
                log(Info) << "TrajectoryActionlib: We are there, poppin'!" << endlog();
            }

            // Check if this was the last point. If so, remove the goal handle
            if (t_info.points.empty())
            {
                log(Info) << "TrajectoryActionlib: Succeeded this goal!" << endlog();
                gh.setSucceeded();
                goal_handles_.erase(goal_handles_.begin());
            }
        }
    }

    // Send references
    for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
        uint partNr = activeBodyparts[j];
        if (allowedBodyparts[partNr-1] == true) {
			
            // Compute the next reference points
            for ( uint i = 0; i < vector_sizes[partNr-1]; i++ ){
                mRefPoints[partNr-1][i] = mRefGenerators[partNr-1][i].generateReference(desiredPos[partNr-1][i], desiredVel[partNr-1][i], desiredAcc[partNr-1][i], InterpolDts[partNr-1], false, InterpolEpses[partNr-1]);
                pos_out[partNr-1][i]=mRefPoints[partNr-1][i].pos;
                vel_out[partNr-1][i]=mRefPoints[partNr-1][i].vel;
                acc_out[partNr-1][i]=mRefPoints[partNr-1][i].acc;
            }

            posoutport[partNr-1].write( pos_out[partNr-1] );
            veloutport[partNr-1].write( vel_out[partNr-1] );
            accoutport[partNr-1].write( acc_out[partNr-1] );
        }
    }
}

// Called by rtt_action_server_ when a new goal is received
void TrajectoryActionlib::goalCallback(GoalHandle gh) {
    // Accept/reject goal requests here

    log(Info) << "TrajectoryActionlib: Received Message" << endlog();
    uint number_of_goal_joints_ = gh.getGoal()->trajectory.joint_names.size();
    // Loop over all joints within the message received
    uint k = 0;
    bool accept = true;
    int error_code = 0;
    while (k < number_of_goal_joints_) {
        map<string, BodyJointPair>::const_iterator it = joint_map.find(gh.getGoal()->trajectory.joint_names[k]);
        if (it == joint_map.end()) {
            log(Warning) << "TrajectoryActionlib: received a message with joint name ["+gh.getGoal()->trajectory.joint_names[k]+"] that is not listed!" << endlog();
            accept = false;
            error_code = -2;
            k++;
        } else {
            // received a message with joint name that is found. Continue checking for operational state of bodypart
            BodyJointPair bjp = it->second;
            int body_part_id = bjp.first;
            int joint_id = bjp.second;
            if (allowedBodyparts[body_part_id] == true) {
                if ( gh.getGoal()->trajectory.points[0].positions[k] < minpos[body_part_id][joint_id] ) {
                    log(Warning) << "TrajectoryActionlib: Received goal " << gh.getGoal()->trajectory.points[0].positions[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside minimal bound " << minpos [body_part_id][joint_id] << "!" << endlog();
                    accept = false;
                    error_code = -5;
                } else if ( gh.getGoal()->trajectory.points[0].positions[k] > maxpos[body_part_id][joint_id] ) {
                    log(Warning) << "TrajectoryActionlib: Received goal " << gh.getGoal()->trajectory.points[0].positions[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside maximal bound " << maxpos [body_part_id][joint_id] << "!" << endlog();
                    accept = false;
                    error_code = -5;
                }
            } else { // Message received for bodypart that did not get the AllowReadReference!
                log(Warning) << "TrajectoryActionlib: Message received for bodypart that did not get the AllowReadReference!" << endlog();
                accept = false;
                error_code = -1;
            }
            k++;
        }
    }
    if (accept) {
        gh.setAccepted();
        log(Info)<<"TrajectoryActionlib: Accepted goal"<<endlog();
        TrajectoryInfo t_info(gh);

        // Push back goal handle
        goal_handles_.push_back(t_info);
    }
    else {
        Result result;
        result.error_code = error_code;
        gh.setRejected(result, "blaat");
        log(Info)<<"TrajectoryActionlib: Rejected goal"<<endlog();
    }
}

// Called by rtt_action_server_ when a goal is cancelled / preempted
void TrajectoryActionlib::cancelCallback(GoalHandle gh)
{
    log(Info) << "TrajectoryActionlib: Cancelling this goal!" << endlog();
    // Find the goalhandle in the goal_handles_ vector
    for(std::vector<TrajectoryInfo>::iterator it = goal_handles_.begin(); it != goal_handles_.end(); ++it)
    {
        if (gh.getGoalID().id == it->goal_handle.getGoalID().id)
        {
            it->goal_handle.setCanceled();
            it = goal_handles_.erase(it);
            return;
        }
    }
}

void TrajectoryActionlib::AddBodyPart(int partNr, strings JointNames)
{
    // Update map
    for (uint l = 0; l < JointNames.size(); l++) {
        joint_map[JointNames[l]] = make_pair(partNr-1, l);
        log(Info) << "TrajectoryActionlib: Updated map for "<< JointNames[l] <<" with pair (Bodypart,JointId) = (" << partNr-1 << "," << l << ")!" << endlog();
    }

    // Initialize various vectors of vectors
    pos_out[partNr-1].assign(JointNames.size(),0.0);
    vel_out[partNr-1].assign(JointNames.size(),0.0);
    acc_out[partNr-1].assign(JointNames.size(),0.0);
    desiredPos[partNr-1].assign(JointNames.size(),0.0);
    desiredVel[partNr-1].assign(JointNames.size(),0.0);
    desiredAcc[partNr-1].assign(JointNames.size(),0.0);
    current_position[partNr-1].assign(JointNames.size(),0.0);
    minpos[partNr-1].assign(JointNames.size(),0.0);
    maxpos[partNr-1].assign(JointNames.size(),0.0);
    maxvel[partNr-1].assign(JointNames.size(),0.0);
    maxacc[partNr-1].assign(JointNames.size(),0.0);
    mRefGenerators[partNr-1].resize(JointNames.size());
    mRefPoints[partNr-1].resize(JointNames.size());

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

    log(Warning) << "TrajectoryActionlib: Total of "<< totalNumberOfJoints <<" joints for " << numberOfBodyparts << " Bodyparts" << endlog();

    // Get the constraints for each joint from the URDF model
    urdf::Model Model;
    Model.initParam("robot_description");

    for (size_t i = 0; i < JointNames.size(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> Joint = Model.getJoint(JointNames[i]);
        minpos[partNr-1][i] = Joint->limits->lower;
        maxpos[partNr-1][i] = Joint->limits->upper;
        maxvel[partNr-1][i] = Joint->limits->velocity;
        maxacc[partNr-1][i] = Joint->limits->effort;

        log(Info) << "TrajectoryActionlib: Bodypart " << partNr << ", Joint " << JointNames[i] << " has these limits: "<< endlog();
        log(Info) << "minpos="<<minpos[partNr-1][i]<<" maxpos="<<maxpos[partNr-1][i]<<" maxvel="<<maxvel[partNr-1][i]<<" maxacc="<<maxacc[partNr-1][i]<<endlog();
    }
}

void TrajectoryActionlib::SendToPos(int partNr, doubles pos)
{
	if (pos.size() != vector_sizes[partNr-1]) {
		log(Warning) << "TrajectoryActionlib: Invalid size of pos/vector_sizes[partNr-1]" << endlog();
	}	
	if (allowedBodyparts[partNr-1] == false) {
		log(Warning) << "TrajectoryActionlib: Received SendToPos for bodypart that is not yet allowed" << endlog();
	}	
	log(Warning)<< "TrajectoryActionlib: Received SendToPos goal: " << pos[0] << "!"<< endlog();
		
	for ( uint joint_id = 0; joint_id < pos.size(); joint_id++ ){
		desiredPos [partNr-1] [joint_id] = min( pos[joint_id]            		, maxpos     [partNr-1][joint_id]);
		desiredPos [partNr-1] [joint_id] = max( minpos [partNr-1] [joint_id]    , desiredPos [partNr-1][joint_id]);
		desiredVel [partNr-1] [joint_id] = maxvel [partNr-1] [joint_id];
		desiredAcc [partNr-1] [joint_id] = maxacc [partNr-1] [joint_id];
	}
			
	log(Info)<< "TrajectoryActionlib: Processed SendToPos goal:" << desiredPos[partNr-1][0] << "!"<< endlog();
	log(Info) << "TrajectoryActionlib:  Allowed:     [" << allowedBodyparts[0] << "," << allowedBodyparts[1] << "," << allowedBodyparts[2] << "," << allowedBodyparts[3] << "," << allowedBodyparts[4] << "]" <<endlog();

    return;
}

void TrajectoryActionlib::ResetReferences(int partNr)
{
	if (partNr < 0 || partNr > maxN ) {
		log(Error) <<"TrajectoryActionlib::ResetReferences: Invalid partNr provided: partNr = " << partNr <<endlog();
		return;
	}
	
    //Set the starting value to the current actual value
    uint N = minpos[partNr-1].size();
    doubles actualPos(N,0.0);
    currentpos_inport[partNr-1].read( actualPos );
    for ( uint i = 0; i < N; i++ ){
       mRefGenerators[partNr-1][i].setRefGen(actualPos[i]);
    }
        
    return;
}

bool TrajectoryActionlib::CheckConnectionsAndProperties()
{
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

        //Set the starting value to the current actual value
        currentpos_inport[partNr-1].read( current_position[partNr-1] );
        for ( uint i = 0; i < vector_sizes[partNr-1]; i++ ){
           mRefGenerators[partNr-1][i].setRefGen(current_position[partNr-1][i]);
        }
    }

    return true;
}

ORO_CREATE_COMPONENT(ROS::TrajectoryActionlib)
