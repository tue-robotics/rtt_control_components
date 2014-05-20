#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>

#include "SafetyMonitor.hpp"

#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

SafetyMonitor::SafetyMonitor(const string& name) : TaskContext(name, PreOperational)
{
    addPort( "jointErrors",jointErrors_inport).doc("Receives joint control errors");
    addPort( "jointAngles",jointAngles_inport).doc("Receives measured relative joint angles");    
    addPort( "controllerOutput",controllerOutput_inport).doc("Receives motorspace output of the controller");
    
    addPort( "homing", homing_inport ).doc("Receives which joint is homing at the moment");
    addPort( "reNull", nulling_inport ).doc("Renull port to receive when nulling is done");
        
    addPort( "safe", safe_outport ).doc("boolean value, safe = true when safe, and safe = false when errors are detected");
    addPort( "resetRef",resetRefPort).doc("Sends reset joint coordinates to ROS topic");
    addPort( "resetInt",resetIntPort).doc("Sends resetvalues to the ReferenceInterpolator");
    addPort( "resetInt2",resetIntPort2).doc("Sends resetvalues to the ReferenceInterpolator");
    
    addProperty( "vectorsize_motorspace", Nm ).doc("An unsigned integer that specifies the size of the motor space");
    addProperty( "vectorsize_jointspace", Nj ).doc("An unsigned integer that specifies the size of the joint space");
    addProperty( "maxJointErrors", MAX_ERRORS).doc("Maximum joint error allowed [rad]");
    addProperty( "motorSaturations", MOTORSAT ).doc("Motor saturation values");
    addProperty( "maxConSatTime", MAXCONSATTIME ).doc("Maximum time the controller is allowed to be saturated");
    addProperty( "partNr", partNr ).doc("partNr for resetting info");
}

SafetyMonitor::~SafetyMonitor(){}

bool SafetyMonitor::configureHook()
{  
	return true;  
}

bool SafetyMonitor::startHook()
{ 
	if (MAX_ERRORS.size()!=Nj || MOTORSAT.size()!=Nm ) {
		log(Error)<<"SafetyMonitor: Parameters missing! Check the sizes of maxJointErrors and motorSaturations."<< endlog();
		return false;
	}	
	if (!jointErrors_inport.connected() || !jointAngles_inport.connected() || !controllerOutput_inport.connected() ) {
		log(Error)<<"SafetyMonitor: One of the input ports is not connected!"<<endlog();
		return false;
	}
	if (!safe_outport.connected() || !resetRefPort.connected() || !resetIntPort.connected() ) {
		log(Error)<<"SafetyMonitor: One of the output ports is not connected!"<<endlog();
		return false;
	}
	
	cntr1 = 0;
	cntr2 = 0;
	counter = 0;
	errors = false;
	reNull = false;
	homingjoint=20;
	
	jointErrors.assign(Nj,0.0);
    timeReachedSaturation.assign(Nm,0.0);
    jointAngles.assign(8,0.0);
    resetdata.assign(32,0.0);
    out_msg.position.assign(7,0.0);
    firstSatInstance.resize(Nm);    
    
    reset();
	return true;
}

void SafetyMonitor::updateHook()
{
	// check if bodypart is nulling, when it is nulling, joint error check is shortly disabled
	if ( nulling_inport.read( reNull ) == NewData ) {
		cntr2 = 0;
	}
	if (reNull) {
		cntr2++;
		if (cntr2 >= 20) {
			reNull=false;
		}
	}
	
	// check if bodypart is homing, when it is nulling, joint error check is shortly disabled
	homing_inport.read(homingjoint);

	// Joint error check
	jointErrors_inport.read(jointErrors);
	if (Nj >= 3) {
		if (counter >= 250 ) {
			counter = 0;
			log(Info)<<"SafetyMonitor: jointErrors: [" << jointErrors[0] << "," << jointErrors[1] << "," << jointErrors[2] << "," << jointErrors[3] << "," << jointErrors[4] << "," << jointErrors[5] << "," << jointErrors[6] << "," << jointErrors[7] << "]" << endlog();
		}
		counter++;
	}
	for ( uint i = 0; i < Nj; i++ ) {
		if( (fabs(jointErrors[i])>MAX_ERRORS[i]) && (i != (uint) homingjoint-1) && (!reNull)) {
			if( errors == false ){
				ROS_ERROR_STREAM( "SafetyMonitor: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). PERA output disabled." );
				log(Error)<<"SafetyMonitor: Error of joint q"<<i+1<<" exceeded limit ("<<MAX_ERRORS[i]<<"). PERA output disabled."<<endlog();
				errors = true;
			}
		}
	}
	
	// Motor Saturation check
	doubles controllerOutputs;
	controllerOutputs.assign(Nm,0.0);
	controllerOutput_inport.read(controllerOutputs);

	long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;

	for(unsigned int i = 0;i<Nm;i++){
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
				ROS_ERROR_STREAM( "SafetyMonitor: Motor output "<<i+1<<" satured too long (absolute "<<MAXCONSATTIME<<" sec above "<<fabs(MOTORSAT[i])<<"). PERA output disabled." );
				log(Error)<<"SafetyMonitor: Motor output "<<i+1<<" satured too long (absolute "<<MAXCONSATTIME<<" sec above "<<fabs(MOTORSAT[i])<<"). PERA output disabled."<<endlog();
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

void SafetyMonitor::reset()
{
		// read relative jointAngles
		jointAngles_inport.read(jointAngles);
		
		// Fill up resetdata and construct out_msg
		for ( uint i = 0; i < 8; i++ ) {
			resetdata[i*4]=1.0;
			resetdata[i*4+1]=jointAngles[i];
			resetdata[i*4+2]=0.0;
			resetdata[i*4+3]=0.0;
		}
		for ( uint i = 0; i < 7; i++ ) {
			out_msg.position[i] = jointAngles[i];
		}
		
		// send reset data
		resetRefPort.write(out_msg);
		resetIntPort.write(resetdata);
		resetIntPort2.write(jointAngles);
		return;
}

ORO_CREATE_COMPONENT(SUPERVISORY::SafetyMonitor)
