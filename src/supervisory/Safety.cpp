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
    // Operations
    addOperation("SetMaxErrors", &Safety::SetMaxErrors, this, OwnThread)
        .doc("Set maximum joint errors")
        .arg("MAX_ERRORs","the maximum joint errors");

    // Ports
    addPort( "jointErrors",jointErrors_inport).doc("Receives joint control errors");   
    addPort( "controlEffort",controleffort_inport).doc("Receives motorspace output of the controller");
    addPort( "safe", safe_outport ).doc("boolean value, safe = true when safe, and safe = false when errors are detected");
    
    // Properties
    addProperty( "NM", NM ).doc("An unsigned integer that specifies the size of the motor space");
    addProperty( "NJ", NJ ).doc("An unsigned integer that specifies the size of the joint space");
    addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
    addProperty( "motorSaturations", MOTORSAT ).doc("Motor saturation values");
    addProperty( "maxConSatTime", MAXCONSATTIME ).doc("Maximum time the controller is allowed to be saturated");
}

Safety::~Safety(){}

bool Safety::configureHook()
{  
    return true;
}

bool Safety::startHook()
{ 
    if (MAX_ERRORS.size()!=NJ || MOTORSAT.size()!=NM ) {
        log(Error)<<"Safety: Parameters missing! Check the sizes of maxJointErrors and motorSaturations."<< endlog();
		return false;
	}	
    if (!jointErrors_inport.connected() || !controleffort_inport.connected() ) {
        log(Error)<<"Safety: One of the input ports is not connected!"<<endlog();
		return false;
	}
    if (!safe_outport.connected() ) {
        log(Error)<<"Safety: One of the output ports is not connected!"<<endlog();
		return false;
	}

    errors = false;
    jointErrors.assign(NJ,0.0);
    timeReachedSaturation.assign(NM,0.0);
    firstSatInstance.resize(NM);
    
	return true;
}

void Safety::updateHook()
{
    // Joint error check
	jointErrors_inport.read(jointErrors);
    for ( uint i = 0; i < NJ; i++ ) {
        if( (fabs(jointErrors[i])>MAX_ERRORS[i]) ) {
			if( errors == false ){
                ROS_ERROR_STREAM( "Safety: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). output disabled." );
				errors = true;
			}
		}
	}
	
	// Motor Saturation check
    doubles controlEffort;
    controlEffort.assign(NM,0.0);
    controleffort_inport.read(controlEffort);
	long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;

    for(unsigned int i = 0;i<NM;i++){
        if(firstSatInstance[i]==0 && fabs(controlEffort[i])>=MOTORSAT[i]){
			timeReachedSaturation[i]=timeNow;
			firstSatInstance[i]=1;
		}
        else if(fabs(controlEffort[i])<MOTORSAT[i]){
			timeReachedSaturation[i]=timeNow;
			firstSatInstance[i]=0;
		}
		if(fabs(timeNow-timeReachedSaturation[i])>=MAXCONSATTIME){
			if(errors==false){ // This check makes sure it is printed only once.
                ROS_ERROR_STREAM( "Safety: Motor output "<<i+1<<" satured too long (absolute "<<MAXCONSATTIME<<" sec above "<<fabs(MOTORSAT[i])<<"). output disabled." );
				errors = true;
			}
		}
	}
	
	if (!errors) {
		safe_outport.write(true);
	}
	else {
		safe_outport.write(false);
	}
}

void Safety::SetMaxErrors( doubles SET_MAX_ERRORS )
{
    if (SET_MAX_ERRORS.size() != MAX_ERRORS.size() ) {
        log(Error) << "Safety: SetMaxErrors: Could not update MAX_ERRORS due to wrongly sized SET_MAX_ERRORS" << endlog();
        return;
    } else {
        // calculate totals
        double total_MAX_ERRORS = 0.0;
        double total_SET_MAX_ERRORS = 0.0;
        for(unsigned int i = 0;i<MAX_ERRORS.size();i++) {
            total_MAX_ERRORS += MAX_ERRORS[i];
            total_SET_MAX_ERRORS += SET_MAX_ERRORS[i];
        }

        if (total_SET_MAX_ERRORS > total_MAX_ERRORS) {
            log(Info) << "Safety: SetMaxErrors: Succesfully increased MAX_ERRORS" << endlog();
        } else if (total_SET_MAX_ERRORS < total_MAX_ERRORS) {
            log(Info) << "Safety: SetMaxErrors: Succesfully decreased MAX_ERRORS" << endlog();
        } else {
            log(Info) << "Safety: SetMaxErrors: updated MAX_ERRORS to same values" << endlog();
        }

        MAX_ERRORS = SET_MAX_ERRORS;
        return;
    }
}


ORO_CREATE_COMPONENT(SUPERVISORY::Safety)
