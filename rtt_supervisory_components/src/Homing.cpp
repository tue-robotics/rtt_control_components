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
}

Homing::~Homing(){}

bool Homing::configureHook()
{
    N = homing_refPos.size();
    ref.resize(N); //N joint(s)
    for( unsigned int j = 0; j < N; j++ )
    {
        ref[j].resize(3); //pos, vel, acc
    }
    
    homing_refPos_t.resize(N);
    homing_refVel_t.resize(N);
    
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
    increased_vel = false;
    movetoconstraint = true;
    movetomidpos = false;
    movetoendpos = false;
    
    if ( require_homing ) {
        TaskContext* Spindle_ReadReferences = this->getPeer( homing_compname + "_ReadReferences");
        if ( ! Spindle_ReadReferences->isRunning() ) {
            log(Error) << homing_compname << "_ReadReferences component is not running yet, please start this component first"<<endlog();
        }
        else {
            Spindle_ReadReferences->stop(); //Disabling reading of references. Will be enabled automagically at the end by the supervisor.
        }
    }
    else { // if require_homing parameter is false, homing component can be stopped immediately since no homing is required
        this->stop(); //Nothing to do
    }
    
	homing_order_t = homing_order[JntNr-1]; // writing of homing reference
	homing_refPos_t [homing_order_t-1] = homing_refPos[homing_order_t-1];
	homing_refVel_t [homing_order_t-1] = homing_refVel[homing_order_t-1];
	
	ref[homing_order_t-1][0] = homing_refPos_t[homing_order_t-1];
	ref[homing_order_t-1][1] = homing_refVel_t[homing_order_t-1];
	ref[homing_order_t-1][2] = 0.0;
	send_new_reference = true;

    uint one = 1;
    status_outport.write(one);

  return true;
}

void Homing::updateHook()
{
    if (send_new_reference == true ) {  // if ref changes, the new ref is sent only once to the reference generator
        ref_outport.write(ref);
        send_new_reference = false;
    }

    if ( movetoconstraint ) // Send the joint to constaint position, check if constraint is met, if constraint is met call homing
    {
        homing_order_t = homing_order[JntNr-1]; // writing of homing reference
        homing_refPos_t [homing_order_t-1] = homing_refPos[homing_order_t-1];
        homing_refVel_t [homing_order_t-1] = homing_refVel[homing_order_t-1];
		
        ref[homing_order_t-1][0] = homing_refPos_t[homing_order_t-1];
        ref[homing_order_t-1][1] = homing_refVel_t[homing_order_t-1];
        ref[homing_order_t-1][2] = 0.0;
	
   		int homing_type_t = homing_type[homing_order_t-1];
        switch (homing_type_t) {
            case 0 : 
            {
            if ((absPos[homing_order_t-1]-homing_absPos[homing_order_t-1] >= 0.0) && (homing_refPos_t[homing_order_t-1] >= 0.0)) { // Invert direction if measured direction is not equal to sent reference direction
                    homing_refPos_t[homing_order_t-1] = homing_refPos_t[homing_order_t-1]*-1.0;
                    send_new_reference = true;
                    log(Warning)<< "Homing of " << homing_body << ": inverted homing direction of joint :" << homing_order_t <<endlog();
                }

                if ((abs(absPos[homing_order_t-1]-homing_absPos[homing_order_t-1]) >= 20.0) && (increased_vel == false)) { // Increase velocity if abs homing point is far away
                    homing_refVel_t[homing_order_t-1] = homing_refVel[homing_order_t-1]*4.0;
                    send_new_reference = true;
                    increased_vel = true;
                    log(Warning)<< "Homing of " << homing_body << ": increased velocity of joint :" << homing_order_t <<endlog();
                }

                if (abs(absPos[homing_order_t-1]-homing_absPos[homing_order_t-1]) <= 1.0)
                {
                    log(Warning)<< "Homing of " << homing_body << ": Absolute sensor reached goal of joint :" << homing_order_t <<endlog(); // set these warnings to info or debug if fully tested with arm
                    HomingConstraintMet = true;
                }
            }
            case 1 : 
            {	
				log(Warning) << "fabs(servoErrors[homing_order_t-1]:" <<  fabs(servoErrors[homing_order_t-1]) <<endlog();
				log(Warning) << "homing_error[homing_order_t-1]:" << homing_error[homing_order_t-1] <<endlog();
                servoError_inport.read(servoErrors);
                if (fabs(servoErrors[homing_order_t-1]) >= homing_error[homing_order_t-1])
                {
                    log(Warning)<< "Homing of " << homing_body << ": Servo error exceeded threshold, endstop reached of joint:" << homing_order_t <<endlog();
                    HomingConstraintMet = true;
                }
                else 
                {
					log(Warning)<< "Homing of " << homing_body << ": Servo error not yet small enough of joint:" << homing_order_t <<endlog();
				}
            }
            case 2 : 
            {
                force_inport.read(forces);
                if (fabs(forces[homing_order_t-1]) >= homing_force[homing_order_t-1]) 
                {
                    log(Warning)<< "Homing of " << homing_body << ": Force Sensor measured threshold value" << homing_order_t <<endlog();
                    HomingConstraintMet = true;
                }
            }
            case 3 : 
            {
				endSwitch_inport.read(endSwitch);
                if (!endSwitch.data) 
                {
                    log(Warning)<< "Homing of " << homing_body << ": Endswitch reached of joint:" << homing_order_t <<endlog();
					HomingConstraintMet = true; 
                }
            }
        }

        if ( HomingConstraintMet ) // after homingconstraint is met, call for homing and sent midpos reference
        {
            // Actually call the services
            StopBodyPart(homing_body);
            ResetEncoders((homing_order_t-1),homing_stroke[homing_order_t-1]);
            StartBodyPart(homing_body);
            log(Warning)<< "Homing of " << homing_body << ": called stop, reset and start of joint:" << homing_order_t <<endlog();

            // send body joint to midpos
            ref[homing_order_t-1][0] = homing_midpos[homing_order_t-1];
            ref[homing_order_t-1][1] = 0.0;
            ref[homing_order_t-1][2] = 0.0;
            ref_outport.write(ref);
            log(Warning)<< "Homing of " << homing_body << ": send to midpos of joint:" << homing_order_t <<endlog();

            HomingConstraintMet = false;
            movetoconstraint = false;
            movetomidpos = true;
        }
    }

    if ( movetomidpos ) 				// Wait for joint to reach midpos (position where it does not interfere with rest of the homing procedure)
    {              
		relPos_inport.read(relPos);        
        if ( fabs(relPos[homing_order_t-1]-homing_midpos[homing_order_t-1]) <= 0.01) {
            JntNr++;
            status_outport.write(JntNr);
            movetomidpos = false;
            if ( JntNr == (N + 1)) // if last joint is homed, go to movetoendpos state
            {
                movetoendpos = true;
            }
            else // go to next joint
            {
                movetoconstraint = true;
            }

            log(Warning)<< "Homing of " << homing_body << ": Midpos reached of joint:" << homing_order_t <<endlog();
        }
    }

    if ( movetoendpos ) 	// if Last Jnt is homed and mid pos is reached for the last joint go to end pos
    {
        for (uint j = 0; j < N; j++){
			ref[j][0] = homing_endpos[j]; 
			ref[j][1] = 0.0;
            ref[j][2] = 0.0;
		}
		
        ref_outport.write(ref);
        
		relPos_inport.read(relPos);
        if ( fabs(relPos[0]-homing_endpos[0]) <= 0.01) {
            uint homingfinished = 10;
            status_outport.write(homingfinished);
            log(Warning)<< "Homing of " << homing_body << ": Finished homing"  <<endlog();

            // Stop this component.
            this->stop();
        }
    }   
    
}

ORO_CREATE_COMPONENT(SUPERVISORY::Homing)
