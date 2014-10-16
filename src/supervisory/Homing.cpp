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
	addPort( "position",pos_inport );
    addPort( "homing_finished", homingfinished_outport );

    addPort( "endswitch", endswitch_inport );
    addPort( "servo_error_in", jointerrors_inport );
    addPort( "abs_pos_in", absPos_inport );
    addPort( "force_in", forces_inport );

	// Properties
    addProperty( "vector_size",     	N               ).doc("Number of joints");
    addProperty( "number_of_outports",  N_outports      ).doc("Number of out ports");
    addProperty( "outport_sizes",  		outport_sizes   ).doc("Outport vector sizes");    
    addProperty( "bodypart",        	bodypart        ).doc("Name of the bodypart, (fill in BODYPARTNAME)");
    addProperty( "prefix",          	prefix          ).doc("Prefix of components (for example: SPINDLE or RPERA)");

    addProperty( "homing_type",     	homing_type     ).doc("Type of homing choose from: ['endswitch','servoerror', 'absolutesensor', 'forcesensor']");    
    addProperty( "require_homing",  	require_homing  ).doc("Vector of boolean values to specify which joints should be homed");
    addProperty( "homing_order",    	homing_order    ).doc("The order in which the joints are homed, for example: array [2 3 1]");
    addProperty( "homing_direction",	homing_direction).doc("Homing direction");
    addProperty( "homing_velocity", 	homing_velocity ).doc("Homing velocities");
    addProperty( "homing_stroke",   	homing_stroke   ).doc("Stroke from endstop to reset position (This distance is provided as delta goal after homing position is reached)");
    addProperty( "reset_stroke",        reset_stroke    ).doc("Stroke from resetposition to zero position (also the position the bodypart will assume when the rest of the joints are homed)");
    addProperty( "homing_endpos",   	homing_endpos   ).doc("position that the body should go to after homing is finished.");

    addProperty( "homing_forces",   	homing_forces    ).doc("Force threshold for force sensor homing");
    addProperty( "homing_errors",   	homing_errors   ).doc("Error threshold for endstop homing");
    addProperty( "homing_absPos",   	homing_absPos   ).doc("Value of the absolute sensor at qi=0 for absolute sensor homing");
}

Homing::~Homing(){}

bool Homing::configureHook()
{
    // Input checks generic
    if (homing_type.size() != N || require_homing.size() != N || homing_order.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_type, require_homing or homing_order does not match vector_size"<<endlog();
        return false;
    }
    if (homing_direction.size() != N || homing_velocity.size() != N || homing_stroke.size() != N || reset_stroke.size() != N || homing_endpos.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_direction, homing_velocity, homing_stroke, reset_stroke or homing_endpos does not match vector_size"<<endlog();
        return false;
    }
    
    // add outports
    for ( uint j = 0; j < N_outports; j++ ) {
        this->addPort( ("refout"+to_string(j+1)), ref_outport[j] ); 
	}

    // Check which types of homing are required 
    endswitchhoming = false;
    errorhoming     = false;
    absolutehoming  = false;
    forcehoming     = false;
    for (uint j = 0; j<N; j++) {
        if (homing_type[j] == 1) {
            endswitchhoming = true;
        } else if (homing_type[j] == 2) {
            errorhoming = true;
        } else if (homing_type[j] == 3) {
            absolutehoming = true;
        } else if (homing_type[j] == 4) {
            forcehoming = true;
        } else {
			log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose 1 for endswitch homing, 2 for servoerror homing, 3 for absolute sensor homing, 4 for force sensor homing"<<endlog();
            return false;
        }
    }

    // Input checks specific for homing types
    if ( (forcehoming && homing_forces.size() != N) || (errorhoming && homing_errors.size() != N) || (absolutehoming && homing_absPos.size() != N) ) {
        log(Error) << prefix <<"_Homing: homing_forces["<< homing_forces.size() <<"], homing_errors["<< homing_errors.size() <<"], homing_absPos["<< homing_absPos.size() <<"] should be size " << N <<"."<<endlog();        
        return false;
    }

	return true;
}

bool Homing::startHook()
{
    // Set variables
    state = 0;
    jointNr = 0;
    cntr = 5000;
    joint_finished = false;
    finished = false;
    homing_stroke_goal = 0.0;

    position.assign(N,0.0);
    ref_out_prev.assign(N,0.0);
    ref_out.assign(N,0.0);
    updated_maxerr.assign(N,0.0);
    updated_minpos.assign(N,0.0);
    updated_maxpos.assign(N,0.0);
    updated_maxvel.assign(N,0.0);
    initial_maxerr.assign(N,0.0);
    initial_minpos.assign(N,0.0);
    initial_maxpos.assign(N,0.0);
    initial_maxvel.assign(N,0.0);

    // Connect Components
    Supervisor = this->getPeer("Supervisor");
    ReadEncoders = this->getPeer( prefix + "_ReadEncoders");
    Safety = this->getPeer( prefix + "_Safety");
    ReferenceGenerator = this->getPeer( prefix + "_ReferenceGenerator");

    // Check Connections
    if ( !Supervisor ) {
        log(Error) << prefix <<"_Homing: Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }
    if ( !ReadEncoders ) {
        log(Error) << prefix <<"_Homing: Could not find :" << prefix << "_ReadEncoders component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }
    if ( !ReferenceGenerator ) {
        log(Error) << prefix <<"_Homing: Could not find :" << prefix << "_ReferenceGenerator component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }
    
    // Fetch Property Acces
    Safety_maxJointErrors = Safety->attributes()->getAttribute("maxJointErrors");
    ReferenceGenerator_minpos = ReferenceGenerator->attributes()->getAttribute("minPosition");
    ReferenceGenerator_maxpos = ReferenceGenerator->attributes()->getAttribute("maxPosition");
	ReferenceGenerator_maxvel = ReferenceGenerator->attributes()->getAttribute("maxVelocity");

    // Check Property Acces
    if (!Safety_maxJointErrors.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxJointErrors of the "<< prefix << "_Safety component."<<endlog();
        return false;
    }
    // Check Property Acces
    if (!ReferenceGenerator_minpos.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property minpos of the "<< prefix << "_ReferenceGenerator component."<<endlog();
        return false;
    }
    // Check Property Acces
    if (!ReferenceGenerator_maxpos.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxvel of the "<< prefix << "_ReferenceGenerator component."<<endlog();
        return false;
    }
    // Check Property Acces
    if (!ReferenceGenerator_maxvel.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxvel of the "<< prefix << "_ReferenceGenerator component."<<endlog();
        return false;
    }
    
    // Fetch Operations
    StartBodyPart = Supervisor->getOperation("StartBodyPart");
    StopBodyPart = Supervisor->getOperation("StopBodyPart");
    ResetEncoder = ReadEncoders->getOperation("reset");

    // Check Operations
    if ( !StartBodyPart.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find Supervisor.StartBodyPart Operation!"<<endlog();
        return false;
    }
    if ( !StopBodyPart.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find Supervisor.StopBodyPart Operation!"<<endlog();
        return false;
    }
    if ( !ResetEncoder.ready() ) {
        log(Error) << prefix <<"_Homing: Could not find :" << prefix << "_ReadEncoder.reset Operation!"<<endlog();
        return false;
    }
    
    // Check homing type specific port connections
    if (endswitchhoming) {
        if (!endswitch_inport.connected()) {
            log(Error) << prefix <<"_Homing: endswitch_inport not connected!"<<endlog();
            return false;
        }
    }
    if (errorhoming) {
        if (!jointerrors_inport.connected()) {
            log(Error) << prefix <<"_Homing: jointerrors_inport not connected!"<<endlog();
            return false;
        }
    }
    if (absolutehoming) {
        if (!absPos_inport.connected()) {
            log(Error) << prefix <<"_Homing: absPos_inport not connected!"<<endlog();
            return false;
        }
    }
    if (forcehoming) {
        if (!forces_inport.connected()) {
            log(Error) << prefix <<"_Homing: forces_inport not connected!"<<endlog();
            return false;
        }
    }
    
    // Increase max errors - Increase maxpos - Decresse maxvel
    initial_maxerr = Safety_maxJointErrors.get();    
    initial_minpos = ReferenceGenerator_minpos.get();
    initial_maxpos = ReferenceGenerator_maxpos.get();
    initial_maxvel = ReferenceGenerator_maxvel.get();
    updated_maxerr = Safety_maxJointErrors.get();
    updated_minpos = ReferenceGenerator_minpos.get();
    updated_maxpos = ReferenceGenerator_maxpos.get();
    updated_maxvel = ReferenceGenerator_maxvel.get();

    for (uint j = 0; j<N; j++) {
        if ( (require_homing[j] != 0) ) {
            updated_maxerr[j] = 2.0*initial_maxerr[j];
            updated_minpos[j] = -50.0;
            updated_maxpos[j] += 50.0;
            updated_maxvel[j] = homing_velocity[j];
        }
    }
    
    Safety_maxJointErrors.set(updated_maxerr);
    ReferenceGenerator_minpos.set(updated_minpos);
    ReferenceGenerator_maxpos.set(updated_maxpos);
    ReferenceGenerator_maxvel.set(updated_maxvel);
		
	return true;
}

void Homing::updateHook()
{
	// Check if finished
    if(jointNr==N) {
        for (uint j = 0; j<N; j++) {
            if (require_homing[j] != 0) {
                if (!finished) {

                    // Reset encoders and send joint to endpos
                    finished = true;

                    log(Warning) << prefix <<"_Homing: Stopping body part" <<endlog();
                    StopBodyPart(bodypart);
                    log(Warning) << prefix <<"_Homing: Stopped body part" <<endlog();

                    for (uint j = 0; j<N; j++) {
                        log(Warning) << prefix <<"_Homing: Resetting Encoder "<< homing_order[j] << " with stroke " << reset_stroke[homing_order[j]-1] << ", j = "<<j<<"!" <<endlog();
                        ResetEncoder(homing_order[j]-1,reset_stroke[homing_order[j]-1]);
                    }

                    log(Warning) << prefix <<"_Homing: Starting body part" <<endlog();
                    StartBodyPart(bodypart);
                    log(Warning) << prefix <<"_Homing: Started body part" <<endlog();

					sendRef(homing_endpos);
					log(Warning) << prefix <<"_Homing: Finished " << bodypart << " homing."<<endlog();
					homingfinished_outport.write(true);
				}
                return;        
            }
            // if no joint was required to home than do not send homing finished
            log(Error) << prefix <<"_Homing: Looped over all joints without at least one joint that required homing. Do not call for homing if in the ops file all required_homing are specified false!"<<endlog();
        }    
    }
	
    // Check if homing is required for this joint else skip this joint
    if (require_homing[homing_order[jointNr]-1] == 0) {

        // Reset parameters 
        updated_maxerr[homing_order[jointNr]-1] = initial_maxerr[homing_order[jointNr]-1];
        updated_minpos[homing_order[jointNr]-1] = initial_minpos[homing_order[jointNr]-1];
        updated_maxpos[homing_order[jointNr]-1] = initial_maxpos[homing_order[jointNr]-1];
        updated_maxvel[homing_order[jointNr]-1] = initial_maxvel[homing_order[jointNr]-1];
        Safety_maxJointErrors.set(updated_maxerr);
        ReferenceGenerator_minpos.set(updated_minpos);
        ReferenceGenerator_maxpos.set(updated_maxpos);
        ReferenceGenerator_maxvel.set(updated_maxvel);

		log(Warning) << prefix <<"_Homing: Skipped homing of joint "<< homing_order[jointNr] << ". Proceeding to joint " << homing_order[jointNr+1]<< "!" <<endlog();

        // Go to the next joint and start over
        jointNr++;
        return;
    }

    // Read positions
    pos_inport.read(position);

    if (state == 0) {
        // Update reference
        updateHomingRef(homing_order[jointNr]-1);

        // Check homing criterion
        joint_finished = evaluateHomingCriterion(homing_order[jointNr]-1);
        if (joint_finished) {

            // Send to reset position
            ref_out = position;         
                         
            ref_out[homing_order[jointNr]-1] -= homing_stroke[homing_order[jointNr]-1];
            homing_stroke_goal = ref_out[homing_order[jointNr]-1];
            sendRef(ref_out);
                       
            // Reset parameters            
            updated_minpos[homing_order[jointNr]-1] = initial_minpos[homing_order[jointNr]-1];
            updated_maxpos[homing_order[jointNr]-1] = initial_maxpos[homing_order[jointNr]-1];
            updated_maxvel[homing_order[jointNr]-1] = initial_maxvel[homing_order[jointNr]-1];
            ReferenceGenerator_minpos.set(updated_minpos);
            ReferenceGenerator_maxpos.set(updated_maxpos);
            ReferenceGenerator_maxvel.set(updated_maxvel);
            
            state++;
        }
    }
    if (state == 1) {
        if ( (position[homing_order[jointNr]-1] > (homing_stroke_goal-0.01) ) && (position[homing_order[jointNr]-1] < (homing_stroke_goal+0.01) ) ) {
            
            // set error back to normal value
			updated_maxerr[homing_order[jointNr]-1] = initial_maxerr[homing_order[jointNr]-1];
			Safety_maxJointErrors.set(updated_maxerr);

            log(Warning) << prefix <<"_Homing: Finished homing of joint "<< homing_order[jointNr] << ". Proceeding to joint " << homing_order[jointNr+1]<< "!" <<endlog();
            jointNr++;
            state = 0;
        }
    }
}

void Homing::updateHomingRef( uint jointID)
{
    if (homing_type[jointID] != 3 ) {

        // Send to homing position
        ref_out = position;
        ref_out[jointID] = homing_direction[jointID]*25.0;
    } else { // absolute sensor homing

        doubles absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);

        // determine direction using absolute sensor if positive joint direction and positive sensor direction are opposite
        // that can be corrected using the property homing_direction
        double direction = 1.0; // positive unless goal - measured < 0
        if ( (( (double) homing_absPos[jointID])-absolutesensoroutput[jointID]) < 0.0 ) {
            direction = -1.0*homing_direction[jointID];
        }

        // Send to homing position
        ref_out = position;
        ref_out[jointID] = direction*25.0;
    }

    // Write ref if a new ref has been generated
    if (ref_out_prev[jointID] != ref_out[jointID]) {
        ref_out_prev = ref_out;
        sendRef(ref_out);
    }

    return;
}

bool Homing::evaluateHomingCriterion( uint jointID)
{
    bool result = false;

    if (homing_type[jointID] == 1 ) {
        std_msgs::Bool endswitch_msg;
        endswitch_inport.read(endswitch_msg);
        result = !endswitch_msg.data;
    } else if (homing_type[jointID] == 2 ) {
        doubles jointerrors;
        jointerrors_inport.read(jointerrors);
        if (fabs(jointerrors[jointID]) > homing_errors[jointID]) {
            result = true;
        }
    } else if (homing_type[jointID] == 3 ) {
        doubles absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);
        if (absolutesensoroutput[jointID] == (double) homing_absPos[jointID]) {
            result = true;
        }
    } else if (homing_type[jointID] == 4 ) {
        doubles forces;
        forces_inport.read(forces);
        if (forces[jointID] > homing_forces[jointID]) {
            result = true;
        }
    } else {
		if (cntr > 5000) {
			cntr=0;		
			log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose 1 for endswitch homing, 2 for servoerror homing, 3 for absolute sensor homing, 4 for force sensor homing"<<endlog();
		}
		cntr++;
    }

    if (result == true ) {
        log(Warning) << prefix <<"_Homing: Homing Position reached for joint " << jointID + 1 << "."<<endlog();
    }

    return result;
}

void Homing::sendRef(doubles output_total)
{
	uint m = 0;
    for ( uint n = 0; n < N_outports; n++ ) {
		doubles output(outport_sizes[n], 0.0);
		for ( uint p = 0; p < outport_sizes[n]; p++ ) {
			output[p] = output_total[m];
			m++;
		}
		
		if ((N_outports > 1) && (n == 0)) {
			log(Warning) << prefix <<"_Homing: Sending Ref [" << output[0] << "," << output[1] << "," << output[2] << "," << output[3] << "," << output[4] << "," << output[5] << "," << output[6] << "]" <<endlog();
		}
		if ((N_outports > 1) && (n == 1)) {
			log(Warning) << prefix <<"_Homing: Sending Ref [" << output[0] << "]" <<endlog();
		}
				
        ref_outport[n].write(output);
	}
	
	return;
}

void Homing::stopHook() 
{
	// To do: add reset of all parameters to avoid problems when homing component is stopped during homing
}

ORO_CREATE_COMPONENT(SUPERVISORY::Homing)
