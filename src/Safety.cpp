#include "Safety.hpp"

using namespace std;
using namespace RTT;
using namespace SAFETY;

Safety::Safety(const string& name) : TaskContext(name, PreOperational)
{
    //! Ports
    addPort( "jointErrors",jointErrors_inport).doc("Receives joint control errors");
    addEventPort( "controlEffort",controleffort_inport).doc("Receives motorspace output of the controller");
    addPort( "enable", enable_outport ).doc("boolean value, enable = true when enabled, and enable = false when errors are detected");
    addPort( "error", error_outport ).doc("boolean value, error = true when in error, and error = false when no errors are detected");

    //! Attrributes
    addAttribute( "maxJointErrors", maxErrors );

    //! Properties
    addProperty( "prefix", prefix ).doc("Prefix of components (for example: SPINDLE or RPERA)");
    addProperty( "partNr", partNr ).doc("PartNr");
    addProperty( "Nm", Nm ).doc("An unsigned integer that specifies the size of the motor space");
    addProperty( "Nj", Nj ).doc("An unsigned integer that specifies the size of the joint space");
    addProperty( "maxJointErrors", maxErrors).doc("Maximum joint error allowed [rad]");
    addProperty( "motorSaturations", motorSat ).doc("Motor saturation values");
    addProperty( "maxConSatTime", maxConSatTime ).doc("Maximum time the controller is allowed to be saturated");
    addProperty( "additional_safeties", add_safety_portnames).doc("Array of strings containing additional safety names.");

    //! Init TaskContext pointers as Null pointer
    TrajectoryActionlib = NULL;
}

Safety::~Safety()
{
	//! Set TaskContext pointers to NULL;
	TrajectoryActionlib = NULL;
	
    //! Remove Operations
    remove("ResetReferences");
}

bool Safety::configureHook()
{  
    //! Check input Properties
    if ( prefix == "" ) {
        log(Warning) << prefix <<"_Safety: prefix is an empty spring. Please set these in your ops file." << partNr << "!"<<endlog();
    }
    if ( partNr > 6 ) {
        log(Error) << prefix <<"_Safety: Could not configure component: invalid partNr: " << partNr << "!"<<endlog();
        return false;
    }
    if ( Nm > 10 ) {
        log(Error) << prefix <<"_Safety: Could not configure component: invalid Nm: " << Nm << "!"<<endlog();
        return false;
    }
    if ( Nj > 10 ) {
        log(Error) << prefix <<"_Safety: Could not configure component: invalid Nj: " << Nj << "!"<<endlog();
        return false;
    }
    if (maxErrors.size()!=Nj || motorSat.size()!=Nm ) {
        log(Error) << prefix <<"_Safety: Could not configure component: maxErrors.size() : " << maxErrors.size()<<  "!=" << Nj << ". Or motorSat.size() : " << motorSat.size()<< "!=" << Nm << "." << endlog();
        return false;
    }
    if (add_safety_portnames.size() > maxN ) {
        log(Error) << prefix <<"_Safety: Could not configure component: invalid add_safety_portnames.size() : " << add_safety_portnames.size() << "!"<<endlog();
        return false;
    }
    for ( uint i = 0; i < add_safety_portnames.size(); i++ ) {
        if (add_safety_portnames[i] == "") {
            log(Error) << prefix <<"_Safety: Could not configure component: add_safety_portnames["<< i <<"] is an empty string. Please set the portname of the additional safety check. If none, do not specify at all."<< endlog();
            return false;
        }
    }

    //! Add an inport for each additional safety component
    for ( uint i=0; i<add_safety_portnames.size(); i++ ){
        addPort( add_safety_portnames[i], addsafety_inports[i] );
    }
       

    //! Connect and check Component
    if ( hasPeer( "TrajectoryActionlib" ) ) {
		TrajectoryActionlib 		= getPeer( "TrajectoryActionlib");
	    if ( !TrajectoryActionlib ) {
            log(Error) << prefix <<"_Safety: Could not configure component: Could not find : TrajectoryActionlib component! Did you add it as Peer in the ops file?"<<endlog();
			return false;
		}
    }
		  

    //! Fetch Reset operation and check operation
    if (TrajectoryActionlib) {

        // Fetch Operation
		ResetReferences = TrajectoryActionlib->getOperation("ResetReferences");
		
        // SetCaller
        ResetReferences.setCaller(TrajectoryActionlib->engine());

        // Check Operation
        if ( !ResetReferences.ready() ) {
            log(Error) << prefix <<"_Safety: Could not configure component: Could not find : TrajectoryActionlib.ResetReferences Operation!"<<endlog();
			return false;
		}
    }
       
    return true;
}

bool Safety::startHook()
{ 
    //! Check Ports
    if (!jointErrors_inport.connected() || !controleffort_inport.connected() ) {
        log(Error) << prefix <<"_Safety: Could not start component: One of the input ports is not connected!"<<endlog();
        return false;
    }
    for ( uint i = 0; i < add_safety_portnames.size(); i++ ) {
        if (!addsafety_inports[i].connected() ) {
            log(Error) << prefix <<"_Safety: Could not start component: addsafety_inports[ " << i << "] is not connected!"<<endlog();
            return false;
        }
    }
    if (!enable_outport.connected() || !error_outport.connected()) {
        log(Error) << prefix <<"_Safety: Could not start component: Error_outport or enable_outport is not connected!"<<endlog();
        return false;
    }


    //! Init
    errors = false;
    errorcntrs.assign(Nj,0);
    jointErrors.assign(Nj,0.0);
    timeReachedSaturation.assign(Nm,0.0);
    firstSatInstance.resize(Nm);
   
    // Reset Reference
    if (TrajectoryActionlib) {
        if (ResetReferences.ready()) {
            ResetReferences(partNr);
        } else {
            log(Error) << prefix <<"_Safety: Could not start component: ResetReferences could not be executed!"<<endlog();
            return false;
        }
	}

    return true;
}

void Safety::updateHook()
{
    // Joint error check
    jointErrors_inport.read(jointErrors);
    for ( uint i = 0; i < Nj; i++ ) {
        if( (fabs(jointErrors[i])>maxErrors[i]) ) {
            if( errors == false && errorcntrs[i] >= 3) {
                log(Error) << prefix <<"_Safety: Error of joint q"<<i+1<<" exceeded limit ("<<maxErrors[i]<<"). jointErrors["<<i<<"] = " << jointErrors[i] << " output disabled." <<endlog();
                errors = true;
            } else if ( errors == false && errorcntrs[i] < 3) {
                errorcntrs[i]++;
            }
        } else if (errorcntrs[i] != 0) {
            errorcntrs[i] = 0;
        }
    }

    // Motor Saturation check
    doubles controlEffort;
    controlEffort.assign(Nm,0.0);
    controleffort_inport.read(controlEffort);
    long double timeNow = os::TimeService::Instance()->getNSecs()*1e-9;

    for(unsigned int i = 0;i<Nm;i++){
        if(firstSatInstance[i]==0 && fabs(controlEffort[i])>=motorSat[i]){
            timeReachedSaturation[i]=timeNow;
            firstSatInstance[i]=1;
        }
        else if(fabs(controlEffort[i])<motorSat[i]){
            timeReachedSaturation[i]=timeNow;
            firstSatInstance[i]=0;
        }
        if(fabs(timeNow-timeReachedSaturation[i])>=maxConSatTime){
            if(errors==false){ // This check makes sure it is printed only once.
                log(Error) << prefix <<"_Safety: Motor output "<<i+1<<" satured too long (absolute "<<maxConSatTime<<" sec above "<<fabs(motorSat[i])<<"). output disabled." <<endlog();
                errors = true;
            }
        }
    }

    // Additional safety checks
    for ( uint i=0; i<add_safety_portnames.size(); i++){
        std_msgs::Bool safe_i;
        safe_i.data = true;
        if ( addsafety_inports[i].read(safe_i) == NewData ){
            if ( !safe_i.data && !errors){
				errors = true;
                log(Error) << prefix <<"_Safety: error in additional safety: "<< add_safety_portnames[i] << ". output disabled." <<endlog();
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

void Safety::stopHook() 
{
    enable_outport.write(false);
}

ORO_CREATE_COMPONENT(SAFETY::Safety)
