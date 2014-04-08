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
  addEventPort( "endswitch_in", endSwitch_inport );
  addPort( "abs_pos_in", absPos_inport );
  addPort( "force_in", force_inport );
  addPort( "servo_error_in", servoError_inport );
  addPort( "ref_out", ref_outport );
  addPort( "relPos_in", relPos_inport );
  addPort( "status_out", status_outport );

  // Creating variables
  addProperty( "homing_body", homing_body );            // name of the body that is stopped/started during homing procedure  (for example: Spindle)
  addProperty( "homing_compname", homing_compname );   	// body name of components (for example: SPINDLE)
  addProperty( "homing_type", homing_type );            // 0 = abs sensor homing 1 = servo error homing  2 = Force sensor homing  3 = endSwitch homing
  addProperty( "homing_order", homing_order );          // The order in which the joints are homed, array [2 3 1] will make sure joint 2, joint 3 and then joint 1 is homed.
  addProperty( "homing_refPos", homing_refPos );        // Points far away in positive or negative direction. Therefore controlls mainly direction. Overuled for abs sensor homing.
  addProperty( "homing_refVel", homing_refVel );        // Homing velocities
  addProperty( "homing_stroke", homing_stroke );        // Stroke from zero point to homing point (encoders are resetted using this value)
  addProperty( "homing_midpos", homing_midpos );        // position that the joint should have during homing. To avoid collisions with other bodies/ itself
  addProperty( "homing_endpos", homing_endpos );        // position that the body should go to after homing is finished.
  addProperty( "homing_absPos", homing_absPos );        // The absolute value at which the sensor should be moving
  addProperty( "homing_force", homing_force );          // Force at which homing position is reached
  addProperty( "homing_error", homing_error );          // Servo error at which homing position is reached
  addProperty( "require_homing", require_homing );      // set to false to disable homing
  addProperty( "fast_step", fast_step );                // fast step
  addProperty( "slow_step", slow_step );                // slow step  
}

Homing::~Homing(){}

bool Homing::configureHook()
{
    N = homing_refPos.size();
    ref.assign(N,0.0);
    endref.assign(N,0.0);
    prevref.assign(N,0.0);
    servoErrors.assign(N,0.0);
    homing_order.assign(N,0.0);
    homing_type.assign(N,0.0);   
    homing_refPos.assign(N,0.0);
    homing_refVel.assign(N,0.0);
    homing_midpos.assign(N,0.0);
    homing_endpos.assign(N,0.0);
    homing_stroke.assign(N,0.0);
    absPos.assign(N,0.0);
    forces.assign(N,0.0);
    relPos.assign(N,0.0);
    homing_absPos.assign(N,0.0);
    homing_force.assign(N,0.0);
    homing_error.assign(N,0.0);    
    homing_refPos_t.assign(N,0.0);
    homing_refVel_t.assign(N,0.0);
    
    // Lookup the Supervisor component.
	TaskContext* Supervisor = this->getPeer("Supervisor");
	if ( !Supervisor ) {
        log(Error) << "Could not find Supervisor component! Did you add it as Peer in the ops file?"<<endlog();
		return false;
    }

    // Lookup operations of peers
	StartBodyPart = Supervisor->getOperation("StartBodyPart");
	if ( !StartBodyPart.ready() ) {
		log(Error) << homing_body << "Could not find Supervisor.StartBodyPart Operation!"<<endlog();
		return false;
	}
	StopBodyPart = Supervisor->getOperation("StopBodyPart");
	if ( !StopBodyPart.ready() ) {
		log(Error) << homing_body << "Could not find Supervisor.StopBodyPart Operation!"<<endlog();
		return false;
    }

    // Lookup the ReadEncoders and the ReadReferences component.
    TaskContext* ReadEncodersComponent = this->getPeer( homing_compname + "_ReadEncoders");
    if ( !ReadEncodersComponent ) {
        log(Error) << "Could not find :" << homing_compname << "_ReadEncoders component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }
    
    TaskContext* ReadReferenceComponent = this->getPeer( homing_compname + "_ReadReferences");		//To Do make generic for
    if ( !ReadReferenceComponent ) {
        log(Error) << "Could not find :" << homing_compname << "_ReadReferences component! Did you add it as Peer in the ops file?"<<endlog();
        return false;
    }

	// Fetch reset function
	ResetEncoders = ReadEncodersComponent->getOperation("reset");
	if ( !ResetEncoders.ready() ) {
		log(Error) << "Could not find :" << homing_body << "_ReadEncoder.reset Operation!"<<endlog();
		return false;
    }
    
	return true;  
}

bool Homing::startHook()
{ 
    JntNr = 1;
    HomJntNr = homing_order[JntNr-1];
    FastHoming = false;
    MoveJoint = true;
    MoveToMidpos = false;
    MoveToEndpos = false;
    HomingConstraintMet = false;
    prevref[0] = 0.0;

    if ( require_homing ) {
        TaskContext* Spindle_ReadReferences = this->getPeer( homing_compname + "_ReadReferences");
        TaskContext* SPINDLE_ReferenceGenerator = this->getPeer( homing_compname + "_ReferenceGenerator");
        if ( ! Spindle_ReadReferences->isRunning() ) {
            log(Error) << homing_compname << "_ReadReferences component is not running yet, please start this component first"<<endlog();
        }
        else {
            Spindle_ReadReferences->stop(); //Disabling reading of references.
            SPINDLE_ReferenceGenerator->stop(); //Disable Reference Generator.
        }
    }
    else { // if require_homing parameter is false, homing component can be stopped immediately since no homing is required
        this->stop(); //Nothing to do
    }
        
    cntr = 0;
    uint one = 1;
    status_outport.write(one);

    return true;
}

void Homing::updateHook()
{
    if ( MoveJoint ) // Send the joint to constaint position, check if constraint is met, if constraint is met call homing
    {
        prevref[HomJntNr-1] = ref[HomJntNr-1];
        if (FastHoming) {
             ref[HomJntNr-1]     = prevref[HomJntNr-1] + fast_step;
        }
        else {
             ref[HomJntNr-1]     = prevref[HomJntNr-1] + slow_step;
        }
        ref_outport.write(ref);
        
		if (cntr >= 1000) {
			log(Warning)<< "Homing of " << homing_body << ": sent reference:" << ref[HomJntNr-1] <<endlog();
			cntr = 0;
		}
		cntr++;
		
        int homing_type_t = homing_type[HomJntNr-1];
        switch (homing_type_t) {
            case 0 : 
            {
            if ((absPos[HomJntNr-1]-homing_absPos[HomJntNr-1] >= 0.0) && (homing_refPos_t[HomJntNr-1] >= 0.0)) { // Invert direction if measured direction is not equal to sent reference direction
                    homing_refPos_t[HomJntNr-1] = homing_refPos_t[HomJntNr-1]*-1.0;
                    log(Warning)<< "Homing of " << homing_body << ": inverted homing direction of joint :" << HomJntNr <<endlog();
                }

                if ((abs(absPos[HomJntNr-1]-homing_absPos[HomJntNr-1]) >= 20.0) && (FastHoming == false)) { // Increase velocity if abs homing point is far away
                    homing_refVel_t[HomJntNr-1] = homing_refVel[HomJntNr-1]*4.0;
                    FastHoming = true;
                    log(Warning)<< "Homing of " << homing_body << ": increased velocity of joint :" << HomJntNr <<endlog();
                }

                if (abs(absPos[HomJntNr-1]-homing_absPos[HomJntNr-1]) <= 1.0)
                {
                    log(Warning)<< "Homing of " << homing_body << ": Absolute sensor reached goal of joint :" << HomJntNr <<endlog(); // set these warnings to info or debug if fully tested with arm
                    HomingConstraintMet = true;
                }
            }
            case 1 : 
            {	
                log(Warning) << "fabs(servoErrors[HomJntNr-1]:" <<  fabs(servoErrors[HomJntNr-1]) <<endlog();
                log(Warning) << "homing_error[HomJntNr-1]:" << homing_error[HomJntNr-1] <<endlog();
                servoError_inport.read(servoErrors);
                if (fabs(servoErrors[HomJntNr-1]) >= homing_error[HomJntNr-1])
                {
                    log(Warning)<< "Homing of " << homing_body << ": Servo error exceeded threshold, endstop reached of joint:" << HomJntNr <<endlog();
                    HomingConstraintMet = true;
                }
                else 
                {
                    log(Warning)<< "Homing of " << homing_body << ": Servo error not yet small enough of joint:" << HomJntNr <<endlog();
				}
            }
            case 2 : 
            {
                force_inport.read(forces);
                if (fabs(forces[HomJntNr-1]) >= homing_force[HomJntNr-1])
                {
                    log(Warning)<< "Homing of " << homing_body << ": Force Sensor measured threshold value" << HomJntNr <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 3 : 
            {
				endSwitch_inport.read(endSwitch);
                if (!endSwitch.data) 
                {
                    log(Warning)<< "Homing of " << homing_body << ": Endswitch reached of joint:" << HomJntNr <<endlog();
					HomingConstraintMet = true; 
                }
            }
        }

        if ( HomingConstraintMet ) // after homingconstraint is met, call for homing and sent midpos reference
        {
            // Actually call the services
            StopBodyPart(homing_body);
            ResetEncoders((HomJntNr-1),homing_stroke[HomJntNr-1]);
            StartBodyPart(homing_body);
            log(Warning)<< "Homing of " << homing_body << ": called stop, reset and start of joint:" << HomJntNr <<endlog();

            HomingConstraintMet = false;
            MoveJoint = false;
            MoveToMidpos = true;
            ref[HomJntNr-1] = homing_stroke[HomJntNr-1];
            TaskContext* Spindle_ReadReferences = this->getPeer( homing_compname + "_ReadReferences");
			TaskContext* SPINDLE_ReferenceGenerator = this->getPeer( homing_compname + "_ReferenceGenerator");
            Spindle_ReadReferences->stop(); //Disabling reading of references.
            SPINDLE_ReferenceGenerator->stop(); //Disabling reading of references.    
        }
    }

    if ( MoveToMidpos ) 	// Wait for joint to reach midpos (position where it does not interfere with rest of the homing procedure)
    {
        endref[HomJntNr-1]  = homing_midpos[HomJntNr-1];
        prevref[HomJntNr-1] = ref[HomJntNr-1];
        if (homing_stroke[HomJntNr-1] > homing_midpos[HomJntNr-1]) {
            ref[HomJntNr-1]     = max((prevref[HomJntNr-1] - fast_step), endref[HomJntNr-1]);
        }
        else {
            ref[HomJntNr-1]     = min((prevref[HomJntNr-1] - fast_step), endref[HomJntNr-1]);
        }

        ref_outport.write(ref);
        
		if (cntr >= 1000) {
			servoError_inport.read(servoErrors);
			log(Warning)<< "Going to midpos " << homing_body << ": sent reference:" << ref[HomJntNr-1] << ". Error is: "<< servoErrors[HomJntNr-1] <<endlog();
			cntr = 0;
		}
		cntr++;
        
		relPos_inport.read(relPos);        
        if ( fabs(relPos[HomJntNr-1]-homing_midpos[HomJntNr-1]) <= 0.01) {
            JntNr++;
            status_outport.write(JntNr);
            MoveToMidpos = false;
            if ( JntNr == (N + 1)) // if last joint is homed, go to MoveToEndpos state
            {
                MoveToEndpos = true;
            }
            else // go to next joint
            {
                MoveJoint = true;
                HomJntNr = homing_order[JntNr-1];
            }

            log(Warning)<< "Homing of " << homing_body << ": Midpos reached of joint:" << HomJntNr <<endlog();
        }
    }

    if ( MoveToEndpos ) 	// if Last Jnt is homed and mid pos is reached for the last joint go to end pos
    {
        for (uint j = 0; j < N; j++){
            endref[j] = homing_endpos[j];
            prevref[j] = ref[j];
            if (homing_midpos[HomJntNr-1] > homing_endpos[HomJntNr-1]) {
                ref[HomJntNr-1]     = max((prevref[HomJntNr-1] - fast_step), endref[HomJntNr-1]);
            }
            else {
                ref[HomJntNr-1]     = min((prevref[HomJntNr-1] - fast_step), endref[HomJntNr-1]);
            }
		}
        ref_outport.write(ref);
        
		if (cntr >= 1000) {
			log(Warning)<< "Going to endpos " << homing_body << ": sent reference:" << ref[HomJntNr-1] <<endlog();
			cntr = 0;
		}
		cntr++;
        
		relPos_inport.read(relPos);
        if ( fabs(relPos[0]-homing_endpos[0]) <= 0.01) {
            uint homingfinished = 10;
            status_outport.write(homingfinished);
            StartBodyPart(homing_body); 
            log(Warning)<< "Homing of " << homing_body << ": Endpos reached of joint:" << HomJntNr <<endlog();
            log(Warning)<< "Homing of " << homing_body << ": Finished homing"  <<endlog();
            // Stop this component.
            this->stop();
        }
    }   
    
}

ORO_CREATE_COMPONENT(SUPERVISORY::Homing)
