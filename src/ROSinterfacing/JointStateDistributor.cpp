
#include <rtt/Component.hpp>
#include "JointStateDistributor.hpp"

using namespace std;
using namespace RTT;
using namespace ROS;

JointStateDistributor::JointStateDistributor(const string& name) :
    TaskContext(name, PreOperational)
{
	// Operations
	addOperation("AddBodyPart", &JointStateDistributor::AddBodyPart, this, OwnThread)
		.doc("Add a body part by specifying its partNr and its jointNames")
		.arg("partNr","The number of the bodypart")              
		.arg("JointNames","The name of joints");

	// Ports
	addEventPort( "in", inport );
}

JointStateDistributor::~JointStateDistributor(){}

bool JointStateDistributor::configureHook()
{
	ConnectionsChecked = true;
	totalNumberOfJoints = 0;
	numberOfBodyparts = 0;
    return true;
}

bool JointStateDistributor::startHook()
{
	if (!inport.connected()) {
		log(Error) << "JointStateDistributor: inport is not connected" << endlog();
		return false;
	}
	
	return true;
}

void JointStateDistributor::updateHook()
{
	// Do a check on all outport connections if a new bodypart is attached.
	if (!ConnectionsChecked) {
		log(Warning) << "JointStateDistributor: checking connections" << endlog();
		for (uint l = 0; l < activeBodyparts.size(); l++) {
			if (!pos_outport[activeBodyparts[l]].connected() && !vel_outport[activeBodyparts[l]].connected() && !eff_outport[activeBodyparts[l]].connected() ) {
				log(Error) << "JointStateDistributor: Bodypart:" << activeBodyparts[l]+1 << "has no connected pos, vel or eff ports" << endlog();
			}
		}
		ConnectionsChecked = true;
	}
	
	// if a new message received
	sensor_msgs::JointState in_msg;
	if (inport.read(in_msg) == NewData) {
		
		log(Info) << "JointStateDistributor: Received Message" << endlog();
		// then loop over all joints within the message received
		uint i = 0;
		while (i < in_msg.position.size()) {
			map<string, BodyJointPair>::const_iterator it = joint_map.find(in_msg.name[i]);
			if (it == joint_map.end()) {
				// received a message with joint name that is not listed
				i++;
			} else {
				// received a message with joint name that is found. update the output and go to the next message.
				BodyJointPair bjp = it->second;
				int body_part_id = bjp.first;
				int joint_id = bjp.second;
				pos_out[body_part_id][joint_id] = in_msg.position[i];
				vel_out[body_part_id][joint_id] = in_msg.position[i];
				eff_out[body_part_id][joint_id] = in_msg.position[i];
				i++;
			}
		}
		
		// All joints are evaluated now write the output
		for (uint l = 0; l < activeBodyparts.size(); l++) {
			pos_outport[activeBodyparts[l]].write(pos_out[activeBodyparts[l]]);
			vel_outport[activeBodyparts[l]].write(vel_out[activeBodyparts[l]]);
			eff_outport[activeBodyparts[l]].write(eff_out[activeBodyparts[l]]);
		}
	}
}

void JointStateDistributor::AddBodyPart(int partNr, strings JointNames)
{
	// Update map
	for (uint l = 0; l < JointNames.size(); l++) {
		joint_map[JointNames[l]] = make_pair(partNr-1, l);
		log(Info) << "JointStateDistributor: Updated map for "<< JointNames[l] <<" with pair (Bodypart,JointId) = (" << partNr-1 << "," << l << ")!" << endlog();
	}
	
	// Set Outputvector size
	pos_out[partNr-1].assign(JointNames.size(),0.0);
	vel_out[partNr-1].assign(JointNames.size(),0.0);
	eff_out[partNr-1].assign(JointNames.size(),0.0);
		
	// Add ports
	addPort( ("pos_out"+to_string(partNr)), pos_outport[partNr-1] );
    addPort( ("vel_out"+to_string(partNr)), vel_outport[partNr-1] );
    addPort( ("eff_out"+to_string(partNr)), eff_outport[partNr-1] );
        
    // Update total numer of joints and present bodyparts 
    totalNumberOfJoints += JointNames.size();
	numberOfBodyparts += 1;
	activeBodyparts.resize(numberOfBodyparts);
	activeBodyparts[numberOfBodyparts-1] = partNr-1;
	ConnectionsChecked = false;

	log(Info) << "JointStateDistributor: Total of "<< totalNumberOfJoints <<" joints for " << numberOfBodyparts << " Bodyparts" << endlog();
}

ORO_CREATE_COMPONENT(ROS::JointStateDistributor)
