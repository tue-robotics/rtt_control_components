#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "Homing.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

Homing::Homing(const string& name) : TaskContext(name, PreOperational)
{
	// Ports
	addPort(	"position",				pos_inport );
	addPort(	"homeswitch",			homeswitch_inport );
	addPort(	"servo_error_in",		jointerrors_inport );
	addPort(	"abs_pos_in",			absPos_inport );
	addPort(	"force_in",				forces_inport );
	addPort(	"homing_finished",		homingfinished_outport );

	// Properties
	new_structure = false;
	addProperty("new_structure",		new_structure	).doc("True -> EtherCATread is used to reset encoders. False BODYPARTNAME_ReadEncoders is used to reset encoders");
	addProperty("vector_size",			N				).doc("Number of joints");
	addProperty("number_of_outports",	N_outports		).doc("Number of out ports");
	addProperty("outport_sizes",		outport_sizes	).doc("Outport vector sizes");
	addProperty("bodypart",				bodypart		).doc("Name of the bodypart, (fill in BODYPARTNAME)");
	addProperty("prefix",				prefix			).doc("Prefix of components (for example: SPINDLE or RPERA)");
	addProperty("partNr",				partNr			).doc("PartNr");
	addProperty("jointnames",			jointnames		).doc("Joint names");

	addProperty("homing_type",			homing_type		).doc("Type of homing choose from: ['homeswitch','servoerror', 'absolutesensor', 'forcesensor']");
	addProperty("require_homing",		require_homing	).doc("Vector of boolean values to specify which joints should be homed");
	addProperty("homing_order",			homing_order	).doc("The order in which the joints are homed, for example: array [2 3 1]");
	addProperty("homing_direction",		homing_direction).doc("Homing direction");
	addProperty("homing_velocity",		homingVel		).doc("Homing velocities");
	addProperty("homing_acceleration",	desiredAcc		).doc("Homing accelerations");
	addProperty("homing_stroke",		homing_stroke	).doc("Stroke from endstop to reset position (This distance is provided as delta goal after homing position is reached)");
	addProperty("reset_stroke",			reset_stroke	).doc("Stroke from resetposition to zero position (also the position the bodypart will assume when the rest of the joints are homed)");
	addProperty("homing_endpos",		homing_endpos	).doc("Position to go to after homing");
	addProperty("require_reset",		require_reset   ).doc("Indicates whether to reset the joint to the current position as soon as the homing criterion has been met");
	addProperty("interpolatorDt",		InterpolDt		).doc("InterpolDt (TS)");
	addProperty("interpolatorEps",		InterpolEps		).doc("InterpolEps");

	addProperty("homing_forces",	homing_forces		).doc("Force threshold for force sensor homing");
	addProperty("homing_errors",	homing_errors		).doc("Error threshold for endstop homing");
	addProperty("homing_absPos",	homing_absPos		).doc("Value of the absolute sensor at qi=0 for absolute sensor homing");

	//! Init TaskContext pointers as Null pointer
	Supervisor = NULL;
	ReadEncoders = NULL;
	Safety = NULL;
	GripperControl = NULL;
	TrajectoryActionlib = NULL;
}

Homing::~Homing()
{
	//! Set TaskContext pointers to NULL;
	Supervisor = NULL;
	ReadEncoders = NULL;
	Safety = NULL;
	GripperControl = NULL;
	TrajectoryActionlib = NULL;

	//! Remove Operations
	remove("StartBodyPart");
	remove("StopBodyPart");
	remove("ResetEncoders");
	remove("Eread_ResetEncoders");
	remove("Readenc_ResetEncoders");
	remove("SendToPos");
}

bool Homing::configureHook()
{
	//! add outports
	for ( uint j = 0; j < N_outports; j++ ) {
		addPort( ("posout"+to_string(j+1)), posoutport[j] );
		addPort( ("velout"+to_string(j+1)), veloutport[j] );
		addPort( ("accout"+to_string(j+1)), accoutport[j] );
	}

	//! Property checks
	// Scalars
	if (N <= 0 || N > 10 ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Invalid N: " << N << "!"<<endlog();
		return false;
	}
	if (N_outports <= 0 || N_outports > 10 ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Invalid N_outports: " << N_outports << "!"<<endlog();
		return false;
	}
	if (partNr <= 0 || partNr > 6 ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Invalid partNr: " << partNr << "!"<<endlog();
		return false;
	}
	if (jointnames.size() <= 0 ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Invalid number of jointnames: " << jointnames.size() << "!"<<endlog();
		return false;
	}
	if (InterpolDt <= 0.0 || InterpolEps <= 0.0 ) {
		log(Error) << prefix <<"_Homing: Could not configure component: InterpolDt or InterpolEps is invalid can't be equal or less than zero: " << partNr << "!"<<endlog();
		return false;
	}
	// Strings
	if (bodypart == "" || prefix == "" ) {
		log(Warning) << prefix <<"_Homing: bodypart of prefix is an empty spring. Please set these in your ops file." << partNr << "!"<<endlog();
	}
	// Vectors
	if ( outport_sizes.size() != N_outports ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Invalid size " << outport_sizes.size() << " of outport_sizes. Should be size: "<< N_outports << "." << partNr << "!"<<endlog();
		return false;
	}
	if (homing_type.size() != N || require_homing.size() != N || homing_order.size() != N ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Size of homing_type ("<<homing_type.size()<<"), require_homing ("<<require_homing.size()<<") or homing_order ("<<homing_order.size()<<") does not match vector_size ("<<N<<")"<<endlog();
		return false;
	}
	if (homing_direction.size() != N || homingVel.size() != N || homing_stroke.size() != N || reset_stroke.size() != N ) {
		log(Error) << prefix <<"_Homing: Could not configure component: Size of homing_direction ("<<homing_direction.size()<<"), homing_velocity ("<<homingVel.size()<<"), homing_acceleration ("<<desiredAcc.size()<<"), homing_stroke ("<<homing_stroke.size()<<"), reset_stroke ("<<reset_stroke.size()<<" or homing_endpos ("<<homing_endpos.size()<<")"<<endlog();
		return false;
	}
	if (desiredAcc.size() != N) {
		log(Error) << prefix <<"_Homing: Could not configure component: Size of desiredAcc "<<desiredAcc.size()<<" is incorrect. Size should be: " << N << "." <<endlog();
		return false;
	}
	if (homing_endpos.size() != outport_sizes[0]) {
		log(Error) << prefix <<"_Homing: Could not configure component: Size of homing_endpos "<<homing_endpos.size()<<" does not match outport_sizes[0] "<<outport_sizes[0]<< "." <<endlog();
		return false;
	}
	if (require_reset.size() == 0) {
		log(Warning) << prefix <<"_Homing: require reset is not set " << endlog();
		require_reset.resize(N, 1);
	}
	else if (require_reset.size() != N) {
		log(Error) << prefix <<"_Homing: Could not configure component: Size of require_reset ("<<require_reset.size()<<") does not match vector_size ("<<N<<")"<<endlog();
		return false;
	}

	//! Check which types of homing are required
	homeswitchhoming = false;
	errorhoming = false;
	absolutehoming = false;
	forcehoming = false;
	for (uint j = 0; j<N; j++) {
		if (homing_type[j] == 1) {
			homeswitchhoming = true;
		} else if (homing_type[j] == 2) {
			errorhoming = true;
		} else if (homing_type[j] == 3) {
			absolutehoming = true;
		} else if (homing_type[j] == 4) {
			forcehoming = true;
		} else {
			log(Error) << prefix <<"_Homing: Could not configure component: Invalid homing type provided. Choose 1 for homeswitch homing, 2 for servoerror homing, 3 for absolute sensor homing, 4 for force sensor homing"<<endlog();
			return false;
		}
	}

	//! Input checks specific for homing types
	if ( (forcehoming && homing_forces.size() != N) || (errorhoming && homing_errors.size() != N) || (absolutehoming && homing_absPos.size() != N) ) {
		log(Error) << prefix <<"_Homing: Could not configure component: homing_forces["<< homing_forces.size() <<"], homing_errors["<< homing_errors.size() <<"], homing_absPos["<< homing_absPos.size() <<"] should be size " << N <<"."<<endlog();
		return false;
	}

	//! Resizing
	mRefGenerators.resize(N);
	mRefPoints.resize(N);
	desiredVel.resize(homingVel.size());

	return true;
}

bool Homing::startHook()
{
	//! Init
	joint_finished = false;
	finishing = false;
	finishingdone = false;
	jointNr = 0;
	stateA = 0;
	stateB = 0;
	homing_stroke_goal = 0.0;
	position.assign(N,0.0);
	desiredPos.assign(N,0.0);
	initial_maxerr.assign(N,0.0);
	updated_maxerr.assign(N,0.0);
	allowedBodyparts.resize(5);
	outpos.resize(N, 0.0);
	outvel.resize(N, 0.0);
	outacc.resize(N, 0.0);
	desiredVel = homingVel;

	//! Check peers and set TaskContext pointers to components
	// Supervisor
	if ( hasPeer( "Supervisor" ) ) {
		Supervisor 		= getPeer( "Supervisor");
	}
	else {
		log(Error) << prefix <<"_Homing: Could not start component: Could not access peer Supervisor" << endlog();
		return false;
	}

	// ReadEncoders
	if (new_structure) {
		if ( hasPeer( "EtherCATread" ) ) {
			EtherCATread 		= getPeer( "EtherCATread" );
		}
		else {
			log(Error) << "Homing: Could not start component: Could not access peer EtherCATread" << endlog();
			return false;
		}		
	} else {
		if ( hasPeer( prefix + "_ReadEncoders" ) ) {
			ReadEncoders 		= getPeer( prefix + "_ReadEncoders" );
		}
		else {
			log(Error) << "Homing: Could not start component: Could not access peer " + prefix + "_ReadEncoders" << endlog();
			return false;
		}	
	}

	// Safety
	if ( hasPeer( prefix + "_Safety" ) ) {
		Safety 		= getPeer( prefix + "_Safety" );
	}
	else {
		log(Error) << prefix <<"_Homing: Could not start component: Could not access peer " + prefix + "_Safety" << endlog();
		return false;
	}

	// TrajectoryActionlib
	if ( hasPeer( "TrajectoryActionlib") )	{
		TrajectoryActionlib = getPeer( "TrajectoryActionlib");
	}
	else if ( hasPeer( "TrajectoryActionlib") )	{
		TrajectoryActionlib = getPeer( "TrajectoryActionlib");
	}
	else {
		log(Error) << prefix <<"_Homing: Could not start component: Could not access peer TrajectoryActionlib" << endlog();
		return false;
	}

	// GripperControl (If LPERA or RPERA)
	if (prefix == "LPERA" || prefix == "RPERA") {
		if ( hasPeer( prefix + "_GripperControl" ) ) {
			GripperControl 		= getPeer( prefix + "_GripperControl" );
		}
		else {
			log(Error) << prefix <<"_Homing: Could not start component: Could not access peer " + prefix + "_GripperControl" << endlog();
			return false;
		}
	} else {
		// Delete taskcontext pointer
		delete GripperControl;
	}

	//! Property Acces
	// Fetch acces
	Safety_maxJointErrors = Safety->attributes()->getAttribute("maxJointErrors");
	TrajectoryActionlib_allowedBodyparts = TrajectoryActionlib->attributes()->getAttribute("allowedBodyparts");

	// Check Acces
	if (!Safety_maxJointErrors.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not gain acces to property maxJointErrors of the "<< prefix << "_Safety component."<<endlog();
		return false;
	}
	if ( !TrajectoryActionlib_allowedBodyparts.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not gain acces to property allowedBodyparts of the TrajectoryActionlib component."<<endlog();
		return false;
	}

	//! Operations
	// Fetch
	StartBodyPart 	= Supervisor->getOperation(			"StartBodyPart");
	StopBodyPart 	= Supervisor->getOperation(			"StopBodyPart");
	if (new_structure) {
		Eread_ResetEncoders 	= EtherCATread->getOperation(		"ResetEncoders");
	} else {
		Readenc_ResetEncoders 	= ReadEncoders->getOperation(		"ResetEncoders");
	}
	ResetReferences	= TrajectoryActionlib->getOperation("ResetReferences");
	SendToPos 		= TrajectoryActionlib->getOperation("SendToPos");
	
	// Set Execution engines
	StartBodyPart.setCaller(	Supervisor->engine());
	StopBodyPart.setCaller(		Supervisor->engine());
	if (new_structure) {
		Eread_ResetEncoders.setCaller(	EtherCATread->engine());
	} else {
		Readenc_ResetEncoders.setCaller(	ReadEncoders->engine());
	}
	ResetReferences.setCaller(	TrajectoryActionlib->engine());
	SendToPos.setCaller(		TrajectoryActionlib->engine());

	// Check Operations
	if ( !StartBodyPart.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not find Supervisor.StartBodyPart Operation!"<<endlog();
		return false;
	}
	if ( !StopBodyPart.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not find Supervisor.StopBodyPart Operation!"<<endlog();
		return false;
	}
	if ( !Eread_ResetEncoders.ready() && !Readenc_ResetEncoders.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not find :(" << prefix << "_ReadEncoder/EtherCATread).ResetEncoders Operation!"<<endlog();
		return false;
	}
	if ( !ResetReferences.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not find : TrajectoryActionlib.ResetReferences Operation!"<<endlog();
		return false;
	}
	if ( !SendToPos.ready() ) {
		log(Error) << prefix <<"_Homing: Could not start component: Could not find : TrajectoryActionlib.SendToPos Operation!"<<endlog();
		return false;
	}

	//! Check port connections
	// Generic
	if (!pos_inport.connected()) {
		log(Error) << prefix <<"_Homing: Could not start component: pos_inport not connected!"<<endlog();
		return false;
	}
	if (!homingfinished_outport.connected()) {
		log(Error) << prefix <<"_Homing: Could not start component: homingfinished_outport not connected!"<<endlog();
		return false;
	}
	for ( uint j = 0; j < N_outports; j++ ) {
		if (!posoutport[j].connected()) {
			log(Error) << prefix <<"_Homing: Could not start component: posoutport[" << j << "] not connected!"<<endlog();
			return false;
		}
	}

	// Homing type specific
	if (homeswitchhoming) {
		if (!homeswitch_inport.connected()) {
			log(Error) << prefix <<"_Homing: Could not start component: homeswitch_inport not connected!"<<endlog();
			return false;
		}
	}
	if (errorhoming) {
		if (!jointerrors_inport.connected()) {
			log(Error) << prefix <<"_Homing: Could not start component: jointerrors_inport not connected!"<<endlog();
			return false;
		}
	}
	if (absolutehoming) {
		if (!absPos_inport.connected()) {
			log(Error) << prefix <<"_Homing: Could not start component: absPos_inport not connected!"<<endlog();
			return false;
		}
	}
	if (forcehoming) {
		if (!forces_inport.connected()) {
			log(Error) << prefix <<"_Homing: Could not start component: forces_inport not connected!"<<endlog();
			return false;
		}
	}

	//! Increase allowed joint errors
	// Fetch initial property values
	initial_maxerr = Safety_maxJointErrors.get();
	
	// Update property values
	updated_maxerr = initial_maxerr;
	for (uint j = 0; j<N; j++) {
		if ( (require_homing[j] != 0) ) {
			updated_maxerr[j] = 2.0*initial_maxerr[j];
		}
	}
	Safety_maxJointErrors.set(updated_maxerr);

	//! Initialize refgen
	pos_inport.read( desiredPos );
	for ( uint i = 0; i < N; i++ ){
		mRefGenerators[i].setRefGen(desiredPos[i]);
	}
		
	return true;
}

void Homing::updateHook()
{		
	// Read positions
	pos_inport.read(position);
	
	// Check if homing of bodypart is finished
	if(jointNr==N) {
		
		if (stateA != 2 ) {	// Stopping and resetting bodypart This loop should be entered only once
			stateA = 2;
			stateB = 1;

			// Stop and Reset Encoders
			log(Info) << prefix <<"_Homing: Stopping Bodypart: " << bodypart << "!"<<endlog();
			StopBodyPart(bodypart); 			
			log(Info) << prefix <<"_Homing: Stopping Bodypart: " << bodypart << "!"<<endlog();
			
			if (new_structure) {
				Eread_ResetEncoders(partNr,1,reset_stroke);
			} else {
				Readenc_ResetEncoders(reset_stroke);
			}
			
		} else if (stateB < 5) { // Wait a bit
			stateB++;
			
		} else if (stateB == 5) { // Starting bodypart, This loop should be entered only once
			stateB++;
						
			// If the component is LPERA or RPERA then the gripperControl component needs to be started
			if (prefix == "LPERA" || prefix == "RPERA") {
				if (!GripperControl->isRunning() )
				{
					GripperControl->start();
				}
			}
		
			// Fetch allowedBodyparts from TrajectoryAction, modify and send back
			allowedBodyparts = TrajectoryActionlib_allowedBodyparts.get();
			allowedBodyparts[partNr-1] = true;
			TrajectoryActionlib_allowedBodyparts.set(allowedBodyparts);
			
			StartBodyPart(bodypart);
			SendToPos(partNr, jointnames, homing_endpos);

			// Finished :)
			string printstring = "[";
			for (uint j = 0; j<N; j++) {
				printstring += to_string(require_homing[j]) + ", ";
			}
			log(Warning) << prefix <<"_Homing: Finished homing of " << bodypart << "!"<<endlog();
			homingfinished_outport.write(true);	
			
		}
		
		return;
	}

	// Check whether homing is required for this joint
	unsigned int joint_index = homing_order[jointNr]-1;
	if (require_homing[joint_index] == 0 && (stateA < 2) ) {
	
		// Go to the next joint and start over
		jointNr++;
		SendRef();

		return;
	}

	if (stateA == 0) { // Move to homing goal and evaluate homing criterion
		updateHomingRef(joint_index);

		joint_finished = evaluateHomingCriterion(joint_index);
		if (joint_finished) {
						
			// Reset Reference generator if desired.
			if (require_reset[joint_index]) {
				mRefGenerators[joint_index].setRefGen(position[joint_index]);
			} else {
			    mRefGenerators[joint_index].setRefGen(mRefPoints[joint_index].pos);
			}

			// Send to reset position
			desiredPos[joint_index] = position[joint_index];
			desiredPos[joint_index] -= homing_stroke[joint_index];
			homing_stroke_goal = desiredPos[joint_index];
			desiredVel[joint_index] = 3*desiredVel[joint_index];
			
			// Reset parameters
			updated_maxerr[joint_index] = initial_maxerr[joint_index];
			
			Safety_maxJointErrors.set(updated_maxerr);
			
			stateA++;
		}
	} else if (stateA == 1) {	// Move to zero position relative to homing position (homing_stroke_goal)
		if ( (position[joint_index] > (homing_stroke_goal-0.01) ) && (position[joint_index] < (homing_stroke_goal+0.01) ) ) {
			jointNr++;
			stateA = 0;
		}
	}

	SendRef();
}

void Homing::SendRef()
{	
	// Send ref
	uint j = 0;
	for ( uint n = 0; n < N_outports; n++ ) {
		for ( uint i = 0; i < outport_sizes[n]; i++ ) {
			mRefPoints[j] = mRefGenerators[j].generateReference(desiredPos[j], desiredVel[j], desiredAcc[j], InterpolDt, false, InterpolEps);
			outpos[i]=mRefPoints[j].pos;
			outvel[i]=mRefPoints[j].vel;
			outacc[i]=mRefPoints[j].acc;
			j ++;
		}
		posoutport[n].write( outpos );
		veloutport[n].write( outvel );
		accoutport[n].write( outacc );
	}
}

void Homing::updateHomingRef( uint jointID)
{
	// Read positions
	pos_inport.read(position);
	desiredPos[jointID] = position[jointID];

	// Update direction for homeswitch data and absolute sensor data
	direction = 1.0*homing_direction[jointID];
	if (homing_type[jointID] == 1 ) {

		homeswitch_inport.read(homeswitch_msg);
		
		direction = 1.0;
		if (homeswitch_msg.data == false) {
			direction = -1.0;
		}
	}
	else if (homing_type[jointID] == 3 ) {
		absPos_inport.read(absolutesensoroutput);

		// determine direction using absolute sensor if positive joint direction and positive sensor direction are opposite
		if ( (( (double) homing_absPos[jointID])-absolutesensoroutput[jointID]) < 0.0 ) {
			direction = -1.0*homing_direction[jointID];
		}
	}

	desiredPos[jointID] = direction*25.0;

	return;
}

bool Homing::evaluateHomingCriterion( uint jointID)
{	
	bool result = false;
	if (homing_type[jointID] == 1 ) {
		std_msgs::Bool homeswitch_msg;
		homeswitch_inport.read(homeswitch_msg);
		result = !homeswitch_msg.data;
	} 
	else if (homing_type[jointID] == 2 ) {
		doubles jointerrors;
		jointerrors_inport.read(jointerrors);
		if (fabs(jointerrors[jointID]) > homing_errors[jointID]) {
			result = true;
		}
	} 
	else if (homing_type[jointID] == 3 ) {
		doubles absolutesensoroutput;
		absPos_inport.read(absolutesensoroutput);
		if (abs(absolutesensoroutput[jointID] - (double) homing_absPos[jointID]) <= 3.3/((double)4095.0) ) {
			result = true;
		}
	} 
	else if (homing_type[jointID] == 4 ) {
		doubles forces;
		forces_inport.read(forces);
		if (forces[jointID] > homing_forces[jointID]) {
			result = true;
		}
	} 
	else {
		log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose 1 for homeswitch homing, 2 for servoerror homing, 3 for absolute sensor homing, 4 for force sensor homing"<<endlog();
	}

	if (result == true ) {
		log(Info) << prefix <<"_Homing: Homing Position reached for joint " << jointID + 1 << "."<<endlog();
	}

	return result;
}

ORO_CREATE_COMPONENT(SUPERVISORY::Homing)
