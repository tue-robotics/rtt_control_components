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
    addPort( "ref_out", ref_outport );
    addPort( "homing_finished", homingfinished_outport );

    addPort( "endswitch", endswitch_inport );
    addPort( "servo_error_in", jointerrors_inport );
    addPort( "abs_pos_in", absPos_inport );
    addPort( "force_in", forces_inport );

	// Properties
    addProperty( "vector_size",     N               ).doc("Number of joints");
    addProperty( "bodypart",        bodypart        ).doc("Name of the bodypart, (fill in BODYPARTNAME)");
    addProperty( "prefix",          prefix          ).doc("Prefix of components (for example: SPINDLE or RPERA)");

    addProperty( "homing_type",     homing_type     ).doc("Type of homing choose from: ['endswitch','servoerror', 'absolutesensor', 'forcesensor']");    
    addProperty( "require_homing",  require_homing  ).doc("Vector of boolean values to specify which joints should be homed");
    addProperty( "homing_order",    homing_order    ).doc("The order in which the joints are homed, for example: array [2 3 1]");
    addProperty( "homing_direction",homing_direction).doc("Homing direction");
    addProperty( "homing_velocity", homing_velocity ).doc("Homing velocities");
    addProperty( "homing_stroke",   homing_stroke   ).doc("Stroke from homing point to zero positions (encoders are resetted using this value)");
    addProperty( "homing_midpos",   homing_midpos   ).doc("position that the joint should have during homing. To avoid collisions with other bodies/ itself");
    addProperty( "homing_endpos",   homing_endpos   ).doc("position that the body should go to after homing is finished.");

    addProperty( "homing_forces",   homing_forces    ).doc("Force threshold for force sensor homing");
    addProperty( "homing_errors",   homing_errors   ).doc("Error threshold for endstop homing");
    addProperty( "homing_absPos",   homing_absPos   ).doc("Value of the absolute sensor at qi=0 for absolute sensor homing");
}

Homing::~Homing()
{
}

bool Homing::configureHook()
{
    // Input checks generic
    if (homing_type.size() != N || require_homing.size() != N || homing_order.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_type, require_homing or homing_order does not match vector_size"<<endlog();
        return false;
    }
    if (homing_direction.size() != N || homing_velocity.size() != N || homing_stroke.size() != N || homing_midpos.size() != N || homing_endpos.size() != N  ) {
        log(Error) << prefix <<"_Homing: size of homing_direction, homing_velocity, homing_stroke, homing_midpos or homing_endpos does not match vector_size"<<endlog();
        return false;
    }

    // Check which types of homing are required 
    endswitchhoming = false;
    errorhoming     = false;
    absolutehoming  = false;
    forcehoming     = false;
    for (uint j = 0; j<N; j++) {
        if (homing_type[j] == "endswitch") {
            endswitchhoming = true;
        } else if (homing_type[j] == "servoerror") {
            errorhoming = true;
        } else if (homing_type[j] == "absolutesensor") {
            absolutehoming = true;
        } else if (homing_type[j] == "forcesensor") {
            forcehoming = true;
        } else {
            log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose from :['endswitch','servoerror', 'absolutesensor', 'forcesensor']"<<endlog();
            return false;
        }
    }

    // Input checks specific for homing types
    if ( (forcehoming && homing_forces.size() != N) || (errorhoming && homing_errors.size() != N) || (absolutehoming && homing_absPos.size() != N) ) {
        log(Error) << prefix <<"_Homing: size of homing_force, homing_error or homing_absPos does not match vector_size"<<endlog();
        return false;
    }

	return true;
}

bool Homing::startHook()
{
    // Set variables
    jointNr = 0;
    joint_finished = false;
    state = 0;

    position.assign(N,0.0);
    ref_out.assign(N,0.0);
    updated_maxerr.assign(N,0.0);
    updated_maxpos.assign(N,0.0);
    updated_maxvel.assign(N,0.0);
    initial_maxerr.assign(N,0.0);
    initial_maxpos.assign(N,0.0);
    initial_maxvel.assign(N,0.0);

    // Connect Components
    Supervisor = this->getPeer("Supervisor");
    ReadEncoders = this->getPeer( prefix + "_ReadEncoders");
    Safety = this->getPeer( prefix + "_Safety");
    ReferenceGenerator = this->getPeer( prefix + "_ReferenceGenerator");

    // Check Connections
    if ( !Supervisor ) {
        log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
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
    ReferenceGenerator_minpos = ReferenceGenerator->attributes()->getAttribute("minpos");
    ReferenceGenerator_maxpos = ReferenceGenerator->attributes()->getAttribute("maxpos");
    ReferenceGenerator_maxvel = ReferenceGenerator->attributes()->getAttribute("maxvel");

    // Check Property Acces
    if (Safety_maxJointErrors.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxJointErrors of the "<< prefix << "_Safety component."<<endlog();
    }
    // Check Property Acces
    if (ReferenceGenerator_maxpos.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxpos of the "<< prefix << "_ReferenceGenerator component."<<endlog();
    }
    // Check Property Acces
    if (ReferenceGenerator_maxvel.ready() ) {
        log(Error) << prefix <<"_Homing: Could not gain acces to property maxvel of the "<< prefix << "_ReferenceGenerator component."<<endlog();
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
        if ( (require_homing[j] == true) ) {
            updated_maxerr[j] = 2.0*initial_maxerr[j];
            updated_minpos[j] -= abs(initial_minpos[j]);
            updated_maxpos[j] += abs(initial_maxpos[j]);
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
    // Check if homing is required for this joint
    if (require_homing[jointNr] == false) {

        // Reset parameters
        updated_maxerr[jointNr] = initial_maxerr[jointNr];
        updated_minpos[jointNr] = initial_minpos[jointNr];
        updated_maxpos[jointNr] = initial_maxpos[jointNr];
        updated_maxvel[jointNr] = initial_maxvel[jointNr];
        Safety_maxJointErrors.set(updated_maxerr);
        ReferenceGenerator_minpos.set(updated_minpos);
        ReferenceGenerator_maxpos.set(updated_maxpos);
        ReferenceGenerator_maxvel.set(updated_maxvel);

        // Go to the next joint and start over
        jointNr++;
        return;
    }

    // Check if last joint is finished
    if(jointNr==N) {
        for (uint j = 0; j<N; j++) {
            if (require_homing[jointNr] == true) {
                log(Warning) << "Homing: Finished " << bodypart << "homing."<<endlog();
                homingfinished_outport.write(true);
                return;        
            }
            // if no joint was required to home than do not send homing finished
            log(Error) << "Homing: Looped over all joints without at least one joint that required homing. Do not call for homing if in the ops file all required_homing are specified false!"<<endlog();
        }    
    }

    // Read positions
    pos_inport.read(position);

    if (state == 0) {
        // Update reference
        updateHomingRef(jointNr);

        // Check homing criterion
        joint_finished = evaluateHomingCriterion(jointNr);
        if (joint_finished) {
            // Reset encoders and send joint to midpos
            StopBodyPart(bodypart);
            ResetEncoder(jointNr,homing_stroke[jointNr]);
            StartBodyPart(bodypart);

            // Reset parameters
            updated_maxerr[jointNr] = initial_maxerr[jointNr];
            updated_minpos[jointNr] = initial_minpos[jointNr];
            updated_maxpos[jointNr] = initial_maxpos[jointNr];
            updated_maxvel[jointNr] = initial_maxvel[jointNr];
            Safety_maxJointErrors.set(updated_maxerr);
            ReferenceGenerator_minpos.set(updated_minpos);
            ReferenceGenerator_maxpos.set(updated_maxpos);
            ReferenceGenerator_maxvel.set(updated_maxvel);

            // Send to middle position
            ref_out = position;
            ref_out[jointNr] = homing_midpos[jointNr];
            ref_outport.write(ref_out);

            state++;
        }
    }
    if (state == 1) {
        if ( (position[jointNr] > (homing_midpos[jointNr]*0.999) ) && (position[jointNr] < (homing_midpos[jointNr]*1.001) ) ) {
            jointNr++;
            state = 0;
        }
    }
}

void Homing::updateHomingRef( uint jointNr)
{
    if (homing_type[jointNr] != "absolutesensor" ) {

        // Send to homing position
        ref_out = position;
        ref_out[jointNr] = homing_direction[jointNr]*25.0;

    } else { // absolute sensor homing

        doubles absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);

        // determine direction using absolute sensor if positive joint direction and positive sensor direction are opposite
        // that can be corrected using the property homing_direction
        double direction = 1.0; // positive unless goal - measured < 0
        if ( (( (double) homing_absPos[jointNr])-absolutesensoroutput[jointNr]) < 0.0 ) {
            direction = -1.0*homing_direction[jointNr];
        }

        // Send to homing position
        ref_out = position;
        ref_out[jointNr] = direction*25.0;
    }

    // Write ref if a new ref has been generated
    if (ref_out_prev != ref_out) {
        ref_out_prev = ref_out;
        ref_outport.write(ref_out);
        log(Warning) << "Homing: sent new ref for analog homing joint " << jointNr << "."<<endlog();
    }

    return;
}

bool Homing::evaluateHomingCriterion( uint jointNr)
{
    bool result = false;

    if (homing_type[jointNr] == "endswitch" ) {
        std_msgs::Bool endswitch_msg;
        endswitch_inport.read(endswitch_msg);
        result = endswitch_msg.data;
    } else if (homing_type[jointNr] == "servoerror" ) {
        doubles jointerrors;
        jointerrors_inport.read(jointerrors);
        if (jointerrors[jointNr] > homing_errors[jointNr]) {
            result = true;
        }
    } else if (homing_type[jointNr] == "absolutesensor" ) {
        doubles absolutesensoroutput;
        absPos_inport.read(absolutesensoroutput);
        if (absolutesensoroutput[jointNr] == (double) homing_absPos[jointNr]) {
            result = true;
        }
    } else if (homing_type[jointNr] == "forcesensor" ) {
        doubles forces;
        forces_inport.read(forces);
        if (forces[jointNr] > homing_forces[jointNr]) {
            result = true;
        }
    } else {
        log(Error) << prefix <<"_Homing: Invalid homing type provided. Choose from :['endswitch','servoerror', 'absolutesensor', 'forcesensor']"<<endlog();
        return false;
    }

    if (result == true ) {
        log(Warning) << "Homing: Homing Position reached for joint " << jointNr << "."<<endlog();
    }

    return result;
}

ORO_CREATE_COMPONENT(SUPERVISORY::Homing)
