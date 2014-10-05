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
    // Ports
    addPort( "jointErrors",jointErrors_inport).doc("Receives joint control errors");   
    addEventPort( "controlEffort",controleffort_inport).doc("Receives motorspace output of the controller");
    addPort( "enable", enable_outport ).doc("boolean value, enable = true when enabled, and enable = false when errors are detected");
    addPort( "error", error_outport ).doc("boolean value, error = true when in error, and error = false when no errors are detected");

    // Attrributes
    addAttribute( "maxJointErrors", MAX_ERRORS );

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
    if (!enable_outport.connected() ) {
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
		enable_outport.write(true);
	}
	else {
		enable_outport.write(false);
		error_outport.write(true);
	}
}

void Safety::stopHook() {
    enable_outport.write(false);
}

ORO_CREATE_COMPONENT(SUPERVISORY::Safety)
