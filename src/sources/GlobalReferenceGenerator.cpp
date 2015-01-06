#include "GlobalReferenceGenerator.hpp"

using namespace std;
using namespace RTT;
using namespace SOURCES;

GlobalReferenceGenerator::GlobalReferenceGenerator(const string& name) : TaskContext(name, PreOperational)
{
	// Operations
    addOperation("AddBodyPart", &GlobalReferenceGenerator::AddBodyPart, this, OwnThread)
		.doc("Add a body part by specifying its partNr and its jointNames")
		.arg("partNr","The number of the bodypart")
        .arg("JointNames","The name of joints");
    addOperation("AllowReadReference", &GlobalReferenceGenerator::AllowReadReference, this)
		.doc("Allow a bodypart to receive references. For example to block when not yet homed")
		.arg("partNr","The number of the bodypart")
		.arg("allowed","wether or not the bodypart is allowed to receieve references");
    addOperation( "ResetReference", &GlobalReferenceGenerator::ResetReference, this, OwnThread )
		.doc("Reset the reference generator to measured current position (used in homing)")
		.arg("partNr","The number of the bodypart");

	// Ports
    addPort(     "in",              inport )            .doc("Inport reading joint state message topic from ROS");
}

GlobalReferenceGenerator::~GlobalReferenceGenerator()
{
    // remove operations
    remove("AddBodyPart");
    remove("AllowReadReference");
    remove("ResetReference");
}

bool GlobalReferenceGenerator::configureHook()
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

bool GlobalReferenceGenerator::startHook()
{
    // Initialize
    start_time = os::TimeService::Instance()->getNSecs()*1e-9;

    // Port connection checks:
    if (!inport.connected()) {
        log(Error) << "GlobalReferenceGenerator: Could not start component: inport is not connected" << endlog();
        return false;
    }

    return true;
}


void GlobalReferenceGenerator::updateHook()
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
    

    // If a new message received
    sensor_msgs::JointState in_msg;
    if (inport.read(in_msg) == NewData) {

        log(Info) << "GlobalReferenceGenerator: Received Message" << endlog();
        // then loop over all joints within the message received
        uint k = 0;
        while (k < in_msg.position.size()) {
            map<string, BodyJointPair>::const_iterator it = joint_map.find(in_msg.name[k]);
            if (it == joint_map.end()) {
                log(Warning) << "GlobalReferenceGenerator: received a message with joint name that is not listed!" << endlog();
                k++;
            } else {
                // received a message with joint name that is found. update the output and go to the next message.
                BodyJointPair bjp = it->second;
                int body_part_id = bjp.first;
                int joint_id = bjp.second;
                if (allowedBodyparts[body_part_id] == true) {
                    if ( minpos[body_part_id][joint_id] == 0.0 && maxpos[body_part_id][joint_id] == 0.0 ) {
                        desiredPos [body_part_id] [joint_id] = in_msg.position[k];

                    } else {
                        desiredPos [body_part_id] [joint_id] = min( in_msg.position[k]                  , maxpos     [body_part_id][joint_id]);
                        desiredPos [body_part_id] [joint_id] = max( minpos [body_part_id] [joint_id]    , desiredPos [body_part_id][joint_id]);

                        if (in_msg.position[k] < (minpos [body_part_id][joint_id] - 0.05) ) {
                            log(Warning) << "GlobalReferenceGenerator: Received goal " << in_msg.position[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside minimal bound " << minpos [body_part_id][joint_id] << "!" << endlog();
                        }
                        if (in_msg.position[k] > (maxpos [body_part_id][joint_id] + 0.05) ) {
                            log(Warning) << "GlobalReferenceGenerator: Received goal " << in_msg.position[k] << " for partNr " << body_part_id+1 << ", joint " << joint_id+1 << ". This is outside maximal bound " << maxpos [body_part_id][joint_id] << "!" << endlog();
                        }
                    }
                    desiredVel [body_part_id] [joint_id] = maxvel [body_part_id] [joint_id];
                    desiredAcc [body_part_id] [joint_id] = maxacc [body_part_id] [joint_id];

                } else { // Message received for bodypart that did not get the AllowReadReference!
					log(Warning) << "GlobalReferenceGenerator: Message received for bodypart that did not get the AllowReadReference!" << endlog();
					desiredPos[body_part_id][joint_id] = current_position[body_part_id][joint_id];
                }
                k++;
            }
        }
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

    //log(Warning) << "GlobalReferenceGenerator: end of UpdateHook" << endlog();
}

void GlobalReferenceGenerator::AddBodyPart(int partNr, strings JointNames)
{
    // Update map
    for (uint l = 0; l < JointNames.size(); l++) {
        joint_map[JointNames[l]] = make_pair(partNr-1, l);
        log(Info) << "GlobalReferenceGenerator: Updated map for "<< JointNames[l] <<" with pair (Bodypart,JointId) = (" << partNr-1 << "," << l << ")!" << endlog();
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

    log(Warning) << "GlobalReferenceGenerator: Total of "<< totalNumberOfJoints <<" joints for " << numberOfBodyparts << " Bodyparts" << endlog();
}

// to do do this via property acces instead of function?
void GlobalReferenceGenerator::AllowReadReference(int partNr, bool allowed)
{
	if (allowedBodyparts[partNr-1] != allowed) {
		if (allowed == true ) { log(Warning) << "GlobalReferenceGenerator:  Allowed Reading of References for partNr: "<< partNr <<"!" << endlog();
		} else { log(Warning) << "GlobalReferenceGenerator:  Disabled Reading of References for partNr: "<< partNr <<"!" << endlog(); } 
	}
	
	allowedBodyparts[partNr-1] = allowed;

    return;
}

void GlobalReferenceGenerator::ResetReference(int partNr)
{
    log(Warning) << "GlobalReferenceGenerator: start of ResetReference" << endlog();

    //Set the starting value to the current actual value
    uint N = minpos[partNr-1].size();
    doubles actualPos(N,0.0);
    currentpos_inport[partNr-1].read( actualPos );
    log(Warning) << "GlobalReferenceGenerator: Resettting bodypart " << partNr-1 << " with [" << actualPos[0] << "," << actualPos[1] << ","  << actualPos[2] << ","  << actualPos[3] << ","  << actualPos[4] << ","  << actualPos[5] << ","  << actualPos[6] << ","  << actualPos[7] << "] !"<<endlog();
    for ( uint i = 0; i < N; i++ ){
       mRefGenerators[partNr-1][i].setRefGen(actualPos[i]);
    }

    log(Warning) << "GlobalReferenceGenerator: end of ResetReference" << endlog();

    return;
}

bool GlobalReferenceGenerator::CheckConnectionsAndProperties()
{
    log(Warning) << "GlobalReferenceGenerator: start of CheckConnectionsAndProperties" << endlog();

    // to do use iterator
    for ( uint j = 0; j < activeBodyparts.size(); j++ ) {
        // look up bodypart number
        uint partNr = activeBodyparts[j];
        // Property checks
        if ( (minpos[partNr-1].size() != vector_sizes[partNr-1]) || (maxpos[partNr-1].size() != vector_sizes[partNr-1]) || (maxvel[partNr-1].size() != vector_sizes[partNr-1]) || (maxacc[partNr-1].size() != vector_sizes[partNr-1]) ) {
            log(Error)<<"GlobalReferenceGenerator: Stopping component: Sizes of minpos["<< partNr-1 <<"], maxpos["<< partNr-1 <<"], maxvel["<< partNr-1 <<"], maxacc["<< partNr-1 <<"] -> [" << minpos.size() << "," << maxpos.size() << "," << maxvel.size() << "," << maxacc.size() << "] should be size " << vector_sizes[partNr-1] <<"."<<endlog();
            return false;
        }
        for ( uint i = 0; i < vector_sizes[partNr-1]; i++ ){
            if ( minpos[partNr-1][i] == 0.0 && maxpos[partNr-1][i] == 0.0 ) {
                log(Warning)<<"GlobalReferenceGenerator: minPos and maxPos both specified 0.0. Thus maxPos and minPos boundaries are not taken into account"<<endlog();
            } else if ( minpos[partNr-1][i] > maxpos[partNr-1][i]) {
                log(Error)<<"GlobalReferenceGenerator: Stopping component: minPosition should be specified smaller than maxPosition"<<endlog();
                return false;
            }
            if ( ( maxvel[partNr-1][i] < 0.0) || ( maxacc[partNr-1][i] < 0.0) ) {
                log(Error)<<"GlobalReferenceGenerator: Stopping component: maxVelocity and maxAcceleration should be specified positive"<<endlog();
                return false;
            }
        }
        if (InterpolDts[partNr-1] <= 0.0) {
            log(Error)<<"GlobalReferenceGenerator: Stopping component: interpolDt should be larger than zero for bodypart: "<< partNr-1 << "."<<endlog();
            return false;
        }
        if (InterpolEpses[partNr-1] <= 0.0) {
            log(Error)<<"GlobalReferenceGenerator: Stopping component: InterpolEps should be larger than zero for bodypart: "<< partNr-1 << "."<<endlog();
            return false;
        }

        // Port Connection checks
        if ( !posoutport[partNr-1].connected() && !veloutport[partNr-1].connected() && !accoutport[partNr-1].connected() ) {
            log(Error)<<"GlobalReferenceGenerator: Stopping component: None of the ports: posout" << partNr << ", velout" << partNr << ", accout" << partNr << " is connected."<<endlog();
            return false;
        }
        if ( !currentpos_inport[partNr-1].connected() ) {
            log(Error)<<"GlobalReferenceGenerator: Stopping component: initial_pos" << partNr << " is not connected."<<endlog();
            return false;
        }

        //Set the starting value to the current actual value
        currentpos_inport[partNr-1].read( current_position[partNr-1] );
        for ( uint i = 0; i < vector_sizes[partNr-1]; i++ ){
           mRefGenerators[partNr-1][i].setRefGen(current_position[partNr-1][i]);
        }
    }

    log(Warning) << "GlobalReferenceGenerator: Checked all Properties and ports of GlobalReferenceGenerator" << endlog();

    return true;
}

ORO_CREATE_COMPONENT(SOURCES::GlobalReferenceGenerator)
