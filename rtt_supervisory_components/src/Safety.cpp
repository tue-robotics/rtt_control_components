#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "Safety.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;


Safety::Safety(const string& name) : TaskContext(name, PreOperational)
{
    addPort("errorPort_in",jointErrors_inport).doc("Receives joint control errors");
    addPort("eButtonPort_in",eButton_inport).doc("Receives emergency button signal from Soem");
    addPort("controllerOutputPort_in",controllerOutput_inport).doc("Receives motorspace output of the controller");
    addPort("measRelJointAnglesPort_in",mRelJntAng_inport).doc("Receives measured relative joint angles");
    addPort("homingStatusPort_in",homingStatus_inport).doc("Receives homing status for status board and exclusion of error check for currently homing joint");

    addPort("peraStatusPort_out",peraStatus_outport).doc("For publishing the PERA status to the AMIGO dashboard");
    addPort("enablePort_out",enable_outport).doc("Sends enablesignal to PERA_IO");
    addPort("resetInt_out",resetInt_outport).doc("Reset port for sending references to reference interpolator when emergency button is pressed");
    addPort("resetRef_out",resetRef_outport).doc("Reset port for sending references to ros topic when emergency button is pressed");

    addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
    addProperty( "motorSaturations", MOTORSAT ).doc("Motor saturation values");
    addProperty( "maxConSatTime", MAXCONSATTIME ).doc("Maximum time the controller is allowed to be saturated");
}

Safety::~Safety(){}

bool Safety::configureHook()
{
    jointErrors.assign(8,0.0);
    timeReachedSaturation.assign(8,0.0);
    memset(firstSatInstance, 0, sizeof(firstSatInstance));
    errors=false;
    enable_outport.write(enable);
	return true;  
}

bool Safety::startHook()
{ 
    return true;
}

void Safety::updateHook()
{
    // Read emergency-button state
    eButton_inport.read(eButtonPressed);

    if(!eButtonPressed.data){
        if(pressed!=false){
            log(Info)<<"SAFETY: Emergency Button Released"<<endlog();
        }
        pressed = false;
    }
    else if(eButtonPressed.data){
        if(pressed!=true){
            log(Info)<<"SAFETY: Emergency Button Pressed"<<endlog();
        }
        pressed = true;
    }

    // Read homing status
    homingStatus_inport.read(jntNr);
    if (jntNr == 10) {
        homed = true;
    }

    // Publish Status to Dashboard
    if(homed==true && errors==false) {          // green if homed and no errrors
        statusToDashboard.data = 0;}
    else if(homed==false && errors==false) {    // orange if homing and no errors
        statusToDashboard.data = 1;}
    else if(errors==true) {                     // red if errors
        statusToDashboard.data = 2;}
    peraStatus_outport.write(statusToDashboard);

    if(!pressed){

        /* Set enable to true. In case of an error, it will be set to
         * false. This construction is used to make sure that enable
         * can NEVER become true once an error has occured but CAN become
         * true after unplugging the eButton.
         */
        if(!errors){
            enable = true;
        }

        // check if a controller reaches its saturation value for longer than MAXCONSATTIME

        doubles controllerOutputs;
        controllerOutputs.assign(9,0.0);
        controllerOutput_inport.read(controllerOutputs);

        long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;

        for(unsigned int i = 0;i<7;i++){
            if(firstSatInstance[i]==0 && fabs(controllerOutputs[i])>=MOTORSAT[i]){
                timeReachedSaturation[i]=timeNow;
                firstSatInstance[i]=1;
            }
            else if(fabs(controllerOutputs[i])<MOTORSAT[i]){
                timeReachedSaturation[i]=timeNow;
                firstSatInstance[i]=0;
            }
            if(fabs(timeNow-timeReachedSaturation[i])>=MAXCONSATTIME){
                if(errors==false){ // This check makes sure it is printed only once.
                    log(Error)<<"SAFETY: Motor output "<<i+1<<" satured too long (absolute "<<MAXCONSATTIME<<" sec above "<<fabs(MOTORSAT[i])<<"). PERA output disabled."<<endlog();
                    enable = false;
                    errors = true;
                }
            }
        }

        // Check if joint errors are within specified limits
        for(unsigned int i = 0;i<8;i++){

            if( (fabs(jointErrors[i])>MAX_ERRORS[i]) && (jntNr!=i+1) ){

                enable = false;
                if( errors == false ){
                    log(Error)<<"SAFETY: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). PERA output disabled."<<endlog();
                    errors = true;
                }

            }

        }
    }
    else if(pressed){

        doubles resetdata(32,0.0);
        doubles measRelJntAngles(8,0.0);

        enable = false;
        enable_outport.write(enable);

        // Read angles from PERA angles from IO
        mRelJntAng_inport.read(measRelJntAngles);

        // Fill up resetdata
        for(unsigned int i = 0;i<8;i++){
            resetdata[i*4]=1.0;
            resetdata[i*4+1]=measRelJntAngles[i];
            resetdata[i*4+2]=0.0;
            resetdata[i*4+3]=0.0;
        }

        for ( uint i = 0; i < 7; i++ )
        {
            out_msg.position[i] = measRelJntAngles[i];
        }

        resetInt_outport.write(resetdata); // reset to ref interpolator
        resetRef_outport.write(out_msg); // reset to rostopic
    }

}

ORO_CREATE_COMPONENT(SUPERVISORY::Safety)
