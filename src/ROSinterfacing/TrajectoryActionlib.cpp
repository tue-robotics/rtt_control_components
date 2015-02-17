#include "TrajectoryActionlib.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

TrajectoryActionlib::TrajectoryActionlib(const string& name) : TaskContext(name, PreOperational)
{
    // Operations
    addOperation("AddBodyPart", &TrajectoryActionlib::AddBodyPart, this, OwnThread)
        .doc("Add a body part by specifying its partNr and its jointNames")
        .arg("partNr","The number of the bodypart")
        .arg("JointNames","The name of joints");
    addOperation("AllowReadReference", &TrajectoryActionlib::AllowReadReference, this)
        .doc("Allow a bodypart to receive references. For example to block when not yet homed")
        .arg("partNr","The number of the bodypart")
        .arg("allowed","wether or not the bodypart is allowed to receieve references");
    addOperation( "ResetReference", &TrajectoryActionlib::ResetReference, this, OwnThread )
        .doc("Reset the reference generator to measured current position (used in homing)")
        .arg("partNr","The number of the bodypart");


    // Actionlib
    // Add action server ports to this task's root service
    rtt_action_server_.addPorts(this->provides());

    // Bind action server goal and cancel callbacks
    rtt_action_server_.registerGoalCallback(boost::bind(&TrajectoryActionlib::goalCallback, this, _1));
    rtt_action_server_.registerCancelCallback(boost::bind(&TrajectoryActionlib::cancelCallback, this, _1));

}

TrajectoryActionlib::~TrajectoryActionlib(){
    // remove operations
    remove("AddBodyPart");
    remove("AllowReadReference");
    remove("ResetReference");
}

bool TrajectoryActionlib::configureHook()
{
    // Initialize
    minpos.resize(maxN);
    maxpos.resize(maxN);
    maxvel.resize(maxN);
    maxacc.resize(maxN);

    checked = false;
    totalNumberOfJoints = 0;
    numberOfBodyparts = 0;
    start_time = 0.0;

    allowedBodyparts.resize(maxN);
    vector_sizes.assign(maxN,0);
    InterpolDts.assign(maxN,0.0);
    InterpolEpses.assign(maxN,0.0);

    desiredPos.resize(maxN);
    desiredVel.resize(maxN);
    desiredAcc.resize(maxN);
    pos_out.resize(maxN);
    vel_out.resize(maxN);
    acc_out.resize(maxN);
    current_position.resize(maxN);
    mRefGenerators.resize(maxN);
    mRefPoints.resize(maxN);

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

    // Send references
    for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
        uint partNr = activeBodyparts[j];
        if (allowedBodyparts[partNr-1]) {
            uint partNr = activeBodyparts[j];

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

    // TODO: Put some criteria here
    if ( false ) {
        current_gh_.setSucceeded();
        log(Info)<<"Succeeded new goal"<<endlog();
    }
}

// Called by rtt_action_server_ when a new goal is received
void TrajectoryActionlib::goalCallback(GoalHandle gh) {
    // Accept/reject goal requests here

    current_gh_ = gh;

    log(Info) << "TrajectoryActionlib: Received Message" << endlog();
    uint number_of_goal_joints_ = gh.getGoal()->trajectory.joint_names.size();
    // Loop over all joints within the message received
    uint k = 0;
    bool accept = true;
    while (k < number_of_goal_joints_) {
        map<string, BodyJointPair>::const_iterator it = joint_map.find(gh.getGoal()->trajectory.joint_names[k]);
        if (it == joint_map.end()) {
            log(Warning) << "TrajectoryActionlib: received a message with joint name ["+gh.getGoal()->trajectory.joint_names[k]+"] that is not listed!" << endlog();
            accept = false;
            k++;
        } else {
            // received a message with joint name that is found. update the output and go to the next message.
            BodyJointPair bjp = it->second;
            int body_part_id = bjp.first;
            int joint_id = bjp.second;
            if (allowedBodyparts[body_part_id] == true) {
                if ( minpos[body_part_id][joint_id] == 0.0 && maxpos[body_part_id][joint_id] == 0.0 ) {
                    desiredPos [body_part_id] [joint_id] = gh.getGoal()->trajectory.points[0].positions[k]; // TODO: support more points inside single trajectory

                } else {
                    desiredPos [body_part_id] [joint_id] = min( gh.getGoal()->trajectory.points[0].positions[k]                  , maxpos     [body_part_id][joint_id]);
                    desiredPos [body_part_id] [joint_id] = max( minpos [body_part_id] [joint_id]    , desiredPos [body_part_id][joint_id]);

                    if (gh.getGoal()->trajectory.points[0].positions[k] < (minpos [body_part_id][joint_id] - 0.05) ) {
                        log(Warning) << "TrajectoryActionlib: Received goal " << gh.getGoal()->trajectory.points[0].positions[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside minimal bound " << minpos [body_part_id][joint_id] << "!" << endlog();
                    }
                    if (gh.getGoal()->trajectory.points[0].positions[k] > (maxpos [body_part_id][joint_id] + 0.05) ) {
                        log(Warning) << "TrajectoryActionlib: Received goal " << gh.getGoal()->trajectory.points[0].positions[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside maximal bound " << maxpos [body_part_id][joint_id] << "!" << endlog();
                    }
                }
                desiredVel [body_part_id] [joint_id] = maxvel [body_part_id] [joint_id];
                desiredAcc [body_part_id] [joint_id] = maxacc [body_part_id] [joint_id];

            } else { // Message received for bodypart that did not get the AllowReadReference!
                log(Warning) << "TrajectoryActionlib: Message received for bodypart that did not get the AllowReadReference!" << endlog();
                desiredPos[body_part_id][joint_id] = current_position[body_part_id][joint_id];
            }
            k++;
        }
    }
    if (accept) {
        gh.setAccepted();
        log(Info)<<"Accepted goal"<<endlog();
    }
    else {
        gh.setRejected();
        log(Info)<<"Rejected goal"<<endlog();
    }
}

// Called by rtt_action_server_ when a goal is cancelled / preempted
void TrajectoryActionlib::cancelCallback(GoalHandle gh) {
    // Handle preemption here
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
    addProperty(    "minPos"+to_string(partNr),             minpos[partNr-1] )          .doc("Minimum joint position limit (Homing temporarily supercedes this limits)");
    addProperty(    "maxPos"+to_string(partNr),             maxpos[partNr-1] )          .doc("Maximum joint position limit (Homing temporarily supercedes this limits)");
    addProperty(    "maxVel"+to_string(partNr),             maxvel[partNr-1] )          .doc("Maximum joint velocity limit (Homing temporarily lowers this limit)");
    addProperty(    "maxAcc"+to_string(partNr),             maxacc[partNr-1] )          .doc("Maximum joint acceleration limit");
    addProperty(    "interpolatorDt"+to_string(partNr),     InterpolDts[partNr-1] )     .doc("Interpol Dt");
    addProperty(    "interpolatorEps"+to_string(partNr),    InterpolEpses[partNr-1] )   .doc("Interpol Eps");

    // Update total numer of joints and present bodyparts
    totalNumberOfJoints += JointNames.size();
    vector_sizes[partNr-1] = JointNames.size();
    numberOfBodyparts += 1;
    activeBodyparts.resize(numberOfBodyparts);
    activeBodyparts[numberOfBodyparts-1] = partNr;
    allowedBodyparts[partNr-1] = true;

    log(Warning) << "TrajectoryActionlib: Total of "<< totalNumberOfJoints <<" joints for " << numberOfBodyparts << " Bodyparts" << endlog();
}

// to do do this via property acces instead of function?
void TrajectoryActionlib::AllowReadReference(int partNr, bool allowed)
{
    if (allowedBodyparts[partNr-1] != allowed) {
        if (allowed == true ) { log(Warning) << "TrajectoryActionlib:  Allowed Reading of References for partNr: "<< partNr <<"!" << endlog();
        } else { log(Warning) << "TrajectoryActionlib:  Disabled Reading of References for partNr: "<< partNr <<"!" << endlog(); }
    }

    allowedBodyparts[partNr-1] = allowed;

    return;
}

void TrajectoryActionlib::ResetReference(int partNr)
{
    log(Warning) << "TrajectoryActionlib: start of ResetReference" << endlog();

    //Set the starting value to the current actual value
    uint N = minpos[partNr-1].size();
    doubles actualPos(N,0.0);
    currentpos_inport[partNr-1].read( actualPos );
    log(Warning) << "TrajectoryActionlib: Resettting bodypart " << partNr-1 << " with [" << actualPos[0] << "," << actualPos[1] << ","  << actualPos[2] << ","  << actualPos[3] << ","  << actualPos[4] << ","  << actualPos[5] << ","  << actualPos[6] << ","  << actualPos[7] << "] !"<<endlog();
    for ( uint i = 0; i < N; i++ ){
       mRefGenerators[partNr-1][i].setRefGen(actualPos[i]);
    }

    log(Warning) << "TrajectoryActionlib: end of ResetReference" << endlog();

    return;
}

bool TrajectoryActionlib::CheckConnectionsAndProperties()
{
    log(Warning) << "TrajectoryActionlib: start of CheckConnectionsAndProperties" << endlog();

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

    log(Warning) << "TrajectoryActionlib: Checked all Properties and ports of TrajectoryActionlib" << endlog();

    return true;
}

ORO_CREATE_COMPONENT(ROS::TrajectoryActionlib)
