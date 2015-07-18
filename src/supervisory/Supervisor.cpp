/*
 * Supervisor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 * 	Wrecked by Tim 
 */

#include <rtt/Component.hpp>
#include "Supervisor.hpp"
#include <ros/ros.h>

enum dashboard_cmd_t {HOMING_CMD = 21, START_CMD = 22, STOP_CMD = 23, RESET_CMD = 24};

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

Supervisor::Supervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    //! Operations
    addOperation("AddAllwaysOnPeer", &Supervisor::AddAllwaysOnPeer, this, OwnThread)
            .doc("Add a peer to the AllwaysOnList, all these components are always on!")
            .arg("peerName","Name of the peer to add to the list");
    addOperation("AddOpOnlyPeer", &Supervisor::AddOpOnlyPeer, this, OwnThread)
            .doc("Add a peer to a AddOpOnlyList, all these components are stopped in states other than Operational")
            .arg("peerName","Name of the peer to add to the list")
            .arg("partNr","The number of the bodypart"); 
    addOperation("AddHomingOnlyPeer", &Supervisor::AddHomingOnlyPeer, this, OwnThread)
            .doc("Add a peer to a HomingOnlyList, all these components stopped in states other than Homing")
            .arg("peerName","Name of the peer to add to the list")
            .arg("partNr","The number of the bodypart"); 
    addOperation("AddEnabledPeer", &Supervisor::AddEnabledPeer, this, OwnThread)
            .doc("Add a peer to a EnabledList, all these components are stopped in states other than Homing or Operational")
            .arg("peerName","Name of the peer to add to the list")
            .arg("partNr","The number of the bodypart");
    addOperation("CreateRobotObject", &Supervisor::CreateRobotObject, this, OwnThread)
            .doc("Create a robot object for the dashboard by specifying a vector of default part names")             
            .arg("robotName","name of the robot")
            .arg("defaultBodyParts","vector of strings containing default body part names");
    addOperation("AddBodyPart", &Supervisor::AddBodyPart, this, OwnThread)
            .doc("Add a body part by specifying its name and its properties")
            .arg("partNr","The number of the bodypart")              
            .arg("partName","The name of the bodypart")
            .arg("homeable","Can you home the part")
            .arg("homingmandatory","Can you start the arm directly without succesfull homing sequence")
            .arg("resettable","If an error occurred, can you reset the error");
    addOperation("StartBodyPart", &Supervisor::StartBodyPart, this, OwnThread)
            .doc("Start a body part")
            .arg("partName","The name of the bodypart");              
    addOperation("StopBodyPart", &Supervisor::StopBodyPart, this, OwnThread)
            .doc("Stop a body part")
            .arg("partName","The name of the bodypart");  
    addOperation("DisplaySupervisoredPeers", &Supervisor::displaySupervisoredPeers, this, ClientThread)
            .doc("Display the list of peers");
    
    //! Properties
	restart_aftererror = false;
	addProperty( "ebuttonorder", ebutton_order );
	addProperty( "restart_aftererror", restart_aftererror );
	        
    //! Ports
	addPort( "rosshutdown", rosshutdownport );
	addPort( "rosetherCATenabled", enabled_rosport );
	addPort( "serialRunning", serialRunningPort ).doc("Serial device running port");
	addPort( "dashboardCmd", dashboardCmdPort ).doc("To receive dashboard commands ");  
	addPort( "hardware_status", hardwareStatusPort ).doc("To send hardware status to dashboard "); 
    addPort( "ebutton_status", ebuttonStatusPort ).doc("To send ebutton status to dashboard ");

    //! Set TaskContext pointers to NULL
    GlobalReferenceGenerator = NULL;
}

Supervisor::~Supervisor()
{
	//! Set TaskContext pointers to NULL
    GlobalReferenceGenerator = NULL;
    for( int j = 0; j < AllwaysOnList.size(); j++ ) {
		AllwaysOnList[j] = NULL;
	}
	for( int i = 0; i < 6; i++ ) {
		for( int j = 0; j < OpOnlyList[i].size(); j++ ) {
			OpOnlyList[i][j] = NULL;
		}
		for( int j = 0; j < HomingOnlyList[i].size(); j++ ) {
			HomingOnlyList[i][j] = NULL;
		}
		for( int j = 0; j < EnabledList[i].size(); j++ ) {
			EnabledList[i][j] = NULL;
		}
	}
	
    //! remove operations
	remove("CreateRobotObject");
	remove("AddBodyPart");	
	remove("AddAllwaysOnPeer");
	remove("AddOpOnlyPeer");
	remove("AddHomingOnlyPeer");
	remove("AddEnabledPeer");	
	remove("StartBodyPart");
	remove("StopBodyPart");
    remove("DisplaySupervisoredPeers");

    //! remove dashboard ros parameters from parameter server
    ros::NodeHandle nh("~");
    nh.deleteParam("dashboard");
    nh.deleteParam("dashboard_list");
}

bool Supervisor::configureHook()
{
    //! Init
    // Scalars
	emergency = false;
	goodToGO = false;
	start_time = 0.0;
	aquisition_time = 0.0;
	error_dected_time = 0.0;
	detected_error = false;
	old_structure = false;

    // Msgs
    rosenabledmsg.data = true;
	rosdisabledmsg.data = false;
	dashboardCmdmsg.data.assign(2, 0.0);	
	StatusStalemsg.level = 0;
	StatusIdlemsg.level = 1;
	StatusOperationalmsg.level = 2;
	StatusHomingmsg.level = 3;
	StatusErrormsg.level = 4;
	hardwareStatusmsg.status.resize(6);
	
    // Vectors
    allowedBodyparts.resize(5);
    for( int j = 0; j < 5; j++ ) {
        allowedBodyparts[j] = false;
    }
    for( int partNr = 0; partNr < 6; partNr++ ) {
		hardwareStatusmsg.status[partNr] = StatusStalemsg; 
		homeableParts[partNr] = false;
		idleDueToEmergencyButton[partNr] = false;
		homedParts[partNr] = false;
		staleParts[partNr] = true; // all parts are stale by default, in Addbodypart function they will be set to false
		bodyParts[partNr] = "";
	}	
	staleParts[0] = false; // whole robot is never stale

    //! check ebutton_order property
	if (ebutton_order.size() < 2 || ebutton_order.size() > 4 ) {
		log(Error) << "Supervisor: Could not configure component, size of ebutton_order should be 2, 3 or 4" << endlog();
		return false;
	}
	number_of_ebuttons = ebutton_order.size();
	emergency_switches.resize(number_of_ebuttons);
	for ( int j = 0; j < number_of_ebuttons; j++ ) {		
		if ( (ebutton_order[j] == "Wireless") || (ebutton_order[j] == "Wired") || (ebutton_order[j] == "Reset") || (ebutton_order[j] == "Endswitch") ) {
			addPort( ("ebutton"+ebutton_order[j]), ebutton_ports[j] );
		} else {
			log(Error) << "Supervisor: Could not configure component, ebutton_order[" << j <<"] does not match one of four possible strings ['Wireless','Wired','Reset','Endswitch']" << endlog();
			return false;
		}
	}

    //! Write rosenabled
    enabled_rosport.write( rosenabledmsg );

	return true;
}

bool Supervisor::startHook()
{
	bodyParts[0] = "all";
	hardwareStatusmsg.status[0].name = bodyParts[0];
	
	// Connection checks
	if ( !serialRunningPort.connected() ) {
		log(Error) << "Supervisor: Could not start component: serialport to check soem is not connected" << endlog();
		return false;
	}
	// ebutton connection checks
	for ( int j = 1; j < ebutton_order.size(); j++ ) {
		if (!ebutton_ports[j].connected()) {
			log(Error) << "Supervisor: Could not start component: ebutton_ports[" << j << "] is not connected" << endlog();
			return false;
		}
	}
	
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	aquisition_time = start_time;
	// Wait untill soem is running
	soem_beckhoff_drivers::EncoderMsg serialRunning;
	while (!(serialRunningPort.read(serialRunning) == NewData) )
	{
		start_time = os::TimeService::Instance()->getNSecs()*1e-9;
		if ( start_time - aquisition_time > 1.0 )
		{
			log(Error) << "Supervisor: No soem heartbeat found" << endlog();
			return false;
		}
	}
	aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;

	// Fetch Property Acces
    if ( this->hasPeer( "GlobalReferenceGenerator") ) {
		GlobalReferenceGenerator = this->getPeer( "GlobalReferenceGenerator");
	}
    else if ( this->hasPeer( "TrajectoryActionlib") ) {
		GlobalReferenceGenerator = this->getPeer( "TrajectoryActionlib");
	}
	else {
		old_structure = true;
	}
	
	if (!old_structure) {

        // Feth acces to GlobalReferenceGenerator attribute allowedBodyparts
		AllowReadReferencesRefGen = GlobalReferenceGenerator->attributes()->getAttribute("allowedBodyparts");

		// Check Property Acces
		if (!AllowReadReferencesRefGen.ready() ) {
			log(Error) << "Supervisor: Could not gain acces to GlobalReferenceGenerator.AllowReadReferences"<<endlog();
			return false;
		}

        // Set all Allowances to false
        for ( int j = 1; j < 6; j++ ) {
            setAllowed(j,false);
        }
	}

    return true;
}

void Supervisor::updateHook()
{
	// Time out for dashboard calls
	if (!goodToGO) {
		aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
	}
	if (!goodToGO && (aquisition_time - start_time > 7.0)) {
		goodToGO = true;
		for( int partNr = 1; partNr < 6; partNr++ ) {
			if (staleParts[partNr] == false) {
				setState(partNr, StatusIdlemsg);
			}
		}	
	}
	// Determine timestamp:
	long double new_time = os::TimeService::Instance()->getNSecs()*1e-9;
  
	soem_beckhoff_drivers::EncoderMsg serialRunning;
	if(serialRunningPort.read(serialRunning) == NewData) {
		aquisition_time = new_time;
	}
	else if ( new_time - aquisition_time > 1.0 ) {
		ROS_ERROR_STREAM("Supervisor: Soem crashed!");
		enabled_rosport.write( rosdisabledmsg );
	} 
	
	// Check if emergency button pressed: (by reading last ebutton which is a AND of all ebuttons)
	if ( ebutton_ports[(number_of_ebuttons-1)].read( emergency_switches[(number_of_ebuttons-1)] ) == NewData ) {
		if ( emergency != emergency_switches[(number_of_ebuttons-1)].data && emergency_switches[(number_of_ebuttons-1)].data ) {
			ROS_INFO_STREAM( "Supervisor: Emergency button pressed, shutting down components online components" );
			for ( int partNr = 1; partNr < 6; partNr++ ) {
				if ( hardwareStatusmsg.status[partNr].level == StatusOperationalmsg.level ) {
					idleDueToEmergencyButton[partNr] = true;
				}
				GoIdle(partNr,hardwareStatusmsg);
			}
		}
		else if ( emergency != emergency_switches[(number_of_ebuttons-1)].data && !emergency_switches[(number_of_ebuttons-1)].data ) {
			ROS_INFO_STREAM( "Supervisor: Emergency button released, restoring components" );
			for ( int partNr = 1; partNr < 6; partNr++ ) {
				if ( ( hardwareStatusmsg.status[partNr].level == StatusIdlemsg.level ) && (idleDueToEmergencyButton[partNr] == true)) {
					GoOperational(partNr,hardwareStatusmsg);
					idleDueToEmergencyButton[partNr] = false;
				}
			}
		}
		emergency = emergency_switches[(number_of_ebuttons-1)].data;
	}
	
	// send ebutton status to dashboard (When button 1 is pressed the status of buttons 1+i is not available anymore)
	diagnostic_msgs::DiagnosticArray ebuttonStatusmsg;
	ebuttonStatusmsg.status.resize(number_of_ebuttons);
	
	bool somebuttonpressed = false;
	for ( int j = 0; j < number_of_ebuttons; j++ ) {
		ebutton_ports[j].read(emergency_switches[j]); 
		ebuttonStatusmsg.status[j].level = 3; // data is by default unavailable
		ebuttonStatusmsg.status[j].name = ebutton_order[j];
		if (emergency_switches[j].data == true && !somebuttonpressed) {
			ebuttonStatusmsg.status[j].level = 1;
			somebuttonpressed = true;
		} else if (emergency_switches[j].data == false && !somebuttonpressed) {
			ebuttonStatusmsg.status[j].level = 0;
		}
	}
	ebuttonStatusPort.write(ebuttonStatusmsg);

	// Read DashboardCmds
	if (goodToGO) {
		if ( dashboardCmdPort.read(dashboardCmdmsg) == NewData ) {
			if (dashboardCmdmsg.data[0] == 0) { // 0 = all bodyparts
                if (dashboardCmdmsg.data[1] == HOMING_CMD && emergency == false ) {
					log(Warning) << "Supervisor: Received Homing request from dashboard for all parts" << endlog();      
                    for ( int partNr = 1; partNr < 6; partNr++ ) {
						if (homeableParts[partNr]) { 
							GoHoming(partNr,hardwareStatusmsg);
						} else { 
							GoOperational(partNr,hardwareStatusmsg);
						}
					}
				}
                if (dashboardCmdmsg.data[1] == START_CMD && emergency == false) {
					log(Warning) << "Supervisor: Received Start request from dashboard for all parts" << endlog(); 
					for ( int partNr = 1; partNr < 6; partNr++ ) {
						GoOperational(partNr,hardwareStatusmsg);
					}
				}
                if (dashboardCmdmsg.data[1] == STOP_CMD && emergency == false) {
					log(Warning) << "Supervisor: Received Stop request from dashboard for all parts" << endlog();      
					for ( int partNr = 1; partNr < 6; partNr++ ) {
						GoIdle(partNr,hardwareStatusmsg);
					}
				}    
			}
			else {  // 1 = base, 2 = spindle, 3 = lpera, 4 = rpera, 5 = head
                if (dashboardCmdmsg.data[1] == HOMING_CMD && emergency == false) {
					log(Warning) << "Supervisor: Received Homing request from dashboard for partNr: [" <<  (int) dashboardCmdmsg.data[0] << "]" << endlog();      
					GoHoming((int) dashboardCmdmsg.data[0],hardwareStatusmsg);
				}
                if (dashboardCmdmsg.data[1] == START_CMD && emergency == false) {
					log(Warning) << "Supervisor: Received Start request from dashboard for partNr: [" << (int) dashboardCmdmsg.data[0] << "]" << endlog(); 
					GoOperational((int) dashboardCmdmsg.data[0],hardwareStatusmsg);
				
				}
                if (dashboardCmdmsg.data[1] == STOP_CMD && emergency == false) {
					log(Warning) << "Supervisor: Received Stop request from dashboard for partNr: [" << (int) dashboardCmdmsg.data[0] << "]" << endlog();  
					GoIdle((int) dashboardCmdmsg.data[0],hardwareStatusmsg);
				}    
                if (dashboardCmdmsg.data[1] == RESET_CMD) {
					log(Warning) << "Supervisor: Received Reset Error request from dashboard for partNr: [" << (int) dashboardCmdmsg.data[0] << "]" << endlog();
					setState(dashboardCmdmsg.data[0], StatusIdlemsg);
				}
			}
		}
	}

	//listen to homingfinished ports, once homingfinished command is received these actions are performed
	for ( int partNr = 1; partNr < 6; partNr++ ) {
		if (homeableParts[partNr] == true ) {
			bool homingfinished;
			if ( homingfinished_port[partNr].read( homingfinished ) == NewData ) {
				log(Info) << "Supervisor: homingfinished is true" << endlog();
				if (homingfinished == true) {
					homedParts[partNr] = true;
					GoOperational(partNr,hardwareStatusmsg);
				}
			}
		}
	}

	//listen to error ports, once component is in error a boolean message is sent such that hardware can be brought into error state and the dashboard can be notified
	for ( int partNr = 0; partNr < 6; partNr++ ) {
		bool error;
		if ( error_port[partNr].read( error ) == NewData ) {
			if (error == true) {
				GoError(partNr,hardwareStatusmsg);
			}
		}
	}

	// send hardware status to dashboard
	if (goodToGO) {
		hardwareStatusPort.write(hardwareStatusmsg);
	}
	
	// Check if autonomous restart of a hardware component is required
	if (restart_aftererror) {
		if (hardwareStatusmsg.status[0].level == StatusErrormsg.level) {
			if (!detected_error) {
				detected_error = true;
				log(Warning) << "Supervisor: Automatic Restart: Error detected. Starting countdown!" << endlog();
				error_dected_time = os::TimeService::Instance()->getNSecs()*1e-9;		
			} else if (os::TimeService::Instance()->getNSecs()*1e-9 - error_dected_time >= 5.0) {	
				for ( int partNr = 1; partNr < 6; partNr++ ) {
					if (hardwareStatusmsg.status[partNr].level == StatusErrormsg.level) {
						log(Warning) << "Supervisor: Automatic Restart: Autonomously Restarting bodypart: " << partNr << " !" << endlog();
						detected_error = false;
					
						// first go Idle then restart bodypart:
						setState(partNr, StatusIdlemsg);
						GoOperational(partNr,hardwareStatusmsg);
					}
				}
			}
		} else {
				detected_error = false;
		}	
	}
    
	// Check for shutdown command
	std_msgs::Bool rosshutdownmsg;
	rosshutdownport.read( rosshutdownmsg );
	if ( rosshutdownmsg.data == true )
	{
		ROS_INFO_STREAM( "Shutdown requested" );
		stop();
	}
}

//-----------------------------------------------------

bool Supervisor::AddPeerCheckList( string peerName, vector<TaskContext*> List )
{    
    if( ! hasPeer(peerName) )
    {
        log(Error) << "Supervisor: " << peerName <<": You can't Supervisor a component that is not your peer !" << endlog();
        return false;
    }
    if( ! getPeer(peerName)->isConfigured() )
    {
        ROS_ERROR_STREAM( "Supervisor: the component " + peerName + " is not yet configured. Therefore I cannot start it. Make you wonder, why would it not be configured?" );
    }
    
	vector<TaskContext*>::iterator i;
	for ( i = List.begin() ; i != List.end() ; i++ )
	{
		TaskContext* tc = (*i);
		if( tc == NULL )
		{
		  log(Error) << "Supervisor: List should not contain null values !" << endlog();
		  return false;
		}
		else
		{
			if( tc->getName() == peerName )
			{
				log(Error) << tc->getName() << "Supervisor: " << peerName <<":  is already in the list !" << endlog();
				return false;
			}
		}
	}

	log(Info) << "Supervisor: " << peerName <<": Peer can be succesfully added" << endlog();
    return true;
}

bool Supervisor::startList( vector<TaskContext*> List )
{
	//ROS_INFO_STREAM("Supervisor: starting startlist");
	vector<TaskContext*>::iterator i;
	for ( i = List.begin() ; i != List.end() ; i++ )
	{
		TaskContext* tc = (*i);
		string componentname = tc->getName();

		if (!tc->isRunning() )
		{
			log(Info) << "Supervisor: " << componentname << ": Starting" << endlog();
			ROS_DEBUG_STREAM("Supervisor: " + componentname + ": Starting");
			log(Debug)<<"Supervisor::starting " << componentname << " at " << os::TimeService::Instance()->getNSecs()*1e-9 - start_time <<endlog();
			tc->start();
			log(Debug)<<"Supervisor::started  " << componentname << " at " << os::TimeService::Instance()->getNSecs()*1e-9 - start_time <<endlog();
		}
		if ( ! tc->isRunning() )
		{
			ROS_WARN_STREAM( "Supervisor: " + componentname + ": Could not be started, trying again." );
			tc->start();
			if ( ! tc->isRunning() )
			{
				ROS_ERROR_STREAM( "Supervisor: " + componentname + ": Could not be started!" );
				return false;
			}
		}
	}
	return true;
}

bool Supervisor::stopList( vector<TaskContext*> List )
{
	
	vector<TaskContext*>::iterator i;
	for ( i = List.begin() ; i != List.end() ; i++ )
	{
	  TaskContext* tc = (*i);
	  string componentname = tc->getName();
	  log(Info) << "Supervisor: " << componentname << ": Stopping" << endlog();
	  log(Debug)<<"Supervisor::stopping " << componentname << " at " << os::TimeService::Instance()->getNSecs()*1e-9 - start_time <<endlog();
	  tc->stop();
	  log(Debug)<<"Supervisor::stopped  " << componentname << " at " << os::TimeService::Instance()->getNSecs()*1e-9 - start_time <<endlog();
	}
	return true;
}

bool Supervisor::StartBodyPart( string partName )
{
	for ( int partNr = 0; partNr < 6; partNr++ )
	{
		string ipartName = bodyParts[partNr];
		if (ipartName.compare(partName) == 0)
		{
			startList( EnabledList[partNr] );
			return true;
		}
	}
	log(Error) << "Supervisor: StartBodyPart: No such bodypart" << endlog();
	return false;
}

bool Supervisor::StopBodyPart( string partName )
{
	for ( int partNr = 0; partNr < 6; partNr++ ) {
		string ipartName = bodyParts[partNr];
		if (ipartName.compare(partName) == 0) {
			stopList( EnabledList[partNr] );
			return true;
		}
	}
	log(Error) << "Supervisor: StopBodyPart: No such bodypart" << endlog();
	return false;
}

bool Supervisor::setState(int partNr, diagnostic_msgs::DiagnosticStatus state)
{
	hardwareStatusmsg.status[partNr] = state;
	if (homedParts[partNr] == true) {
		hardwareStatusmsg.status[partNr].message = "homed";
	}
	hardwareStatusmsg.status[partNr].name = bodyParts[partNr];

	// Update all state
	int max_level = 0;
	for ( int partNr = 1; partNr < 6; partNr++ ) {
		max_level = max((int) hardwareStatusmsg.status[partNr].level,max_level);
	}
	
	hardwareStatusmsg.status[0].level = max_level;	
	
	return true;
}

void Supervisor::setAllowed(int partNr, bool allowed)
{
    if (!old_structure) {
        // Fetch
        allowedBodyparts = AllowReadReferencesRefGen.get();
        // Set
        allowedBodyparts[partNr-1] = false;
        // Set
        AllowReadReferencesRefGen.set(allowedBodyparts);
    }
    return;
}

bool Supervisor::GoOperational(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
	if (staleParts[partNr] == false) {
		if (statusArray.status[partNr].level != StatusErrormsg.level) {				// continue only if not in error state
			stopList( HomingOnlyList[partNr] );
			startList( OpOnlyList[partNr] );
			if (!old_structure) { 
				log(Warning) << "Supervisor::GoOperational: Allowing bodypart:     [" << partNr << "]" <<endlog();
				setAllowed(partNr, true);
			}
			
			if (statusArray.status[partNr].level != StatusHomingmsg.level) {		// if not in homing state, the EnabledList does not need to be restarted
				startList( EnabledList[partNr] );
			}
			setState(partNr, StatusOperationalmsg);
		}
	} else {
		setState(partNr, StatusStalemsg);
	}
	
	return true;
}

bool Supervisor::GoIdle(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
	if (staleParts[partNr] == false) {
		stopList( EnabledList[partNr] );
		stopList( HomingOnlyList[partNr] );
		stopList( OpOnlyList[partNr] );
		
		if (!old_structure) {
			log(Warning) << "Supervisor::GoIdle: Allowing bodypart:     [" << partNr << "]" <<endlog();
			setAllowed(partNr, false);
		}
		
		if (statusArray.status[partNr].level != StatusErrormsg.level) {
			setState(partNr, StatusIdlemsg);
		}
	}
	else {
		setState(partNr, StatusStalemsg);
	}
	
	return true;
}

bool Supervisor::GoHoming(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
	if (staleParts[partNr] == false) {
		if (statusArray.status[partNr].level != StatusErrormsg.level) { // if not from error
			stopList( OpOnlyList[partNr] );
			startList( EnabledList[partNr] );
			startList( HomingOnlyList[partNr] );
			setState(partNr, StatusHomingmsg);
			
			if (!old_structure) {
				log(Warning) << "Supervisor::GoHoming: Allowing bodypart:     [" << partNr << "]" <<endlog();
				setAllowed(partNr, false);
			}
		}
	}
	else {
		setState(partNr, StatusStalemsg);
	}
	
	return true;
}

bool Supervisor::GoError(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
	if (staleParts[partNr] == false) {
		stopList( EnabledList[partNr] );
		stopList( HomingOnlyList[partNr] );
		stopList( OpOnlyList[partNr] );

		if (!old_structure) {
			log(Warning) << "Supervisor::GoError: Allowing bodypart:     [" << partNr << "]" <<endlog();
			setAllowed(partNr, false);
		}
		setState(partNr, StatusErrormsg);
	}
	else {
		setState(partNr, StatusStalemsg);
	}
	return true;
}

bool Supervisor::isEmpty( vector<TaskContext*> List )
{	
	vector<TaskContext*>::iterator i;
	for ( i = List.begin(); i != List.end() ; i++ )
	{
	  TaskContext* tc = (*i);
	  if( tc == NULL )
	  {
		  return true;
	  }
	  else
	  {
		  return false;
	  }
	}
	return true;
}

bool Supervisor::AddAllwaysOnPeer(string peerName )
{
	if ( AddPeerCheckList( peerName, AllwaysOnList ) )
	{
		AllwaysOnList.push_back ( getPeer(peerName) ); 
		getPeer(peerName)->start();
		return true;
	}
	return false;
}

bool Supervisor::AddOpOnlyPeer( string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, OpOnlyList[partNr] ) )
	{
		OpOnlyList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::AddHomingOnlyPeer( string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, HomingOnlyList[partNr] ) )
	{
		HomingOnlyList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::AddEnabledPeer( string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, EnabledList[partNr] ) )
	{
		EnabledList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::CreateRobotObject(string robotName, vector<string> defaultBodyParts )
{
	ros::NodeHandle nh("~");
    nh.setParam("dashboard_list",defaultBodyParts);

    for ( uint i = 0 ; i < defaultBodyParts.size(); i++ ) {
        nh.setParam("dashboard/" + defaultBodyParts[i] + "/name", defaultBodyParts[i] );
        nh.setParam("dashboard/" + defaultBodyParts[i] + "/homeable", false );
        nh.setParam("dashboard/" + defaultBodyParts[i] + "/homingmandatory", true );
        nh.setParam("dashboard/" + defaultBodyParts[i] + "/resettable", false );
    }
}

bool Supervisor::AddBodyPart( int partNr, string partName, bool homeable , bool homingmandatory, bool resettable)
{
	if (homeable) {
		addPort( partName+"_homingfinished", homingfinished_port[partNr] );
	}
    addPort( partName+"_error", error_port[partNr] );
	bodyParts[partNr] = partName;
	homeableParts[partNr] = homeable;
	staleParts[partNr] = false;
	
    ros::NodeHandle nh("~");
    nh.setParam("dashboard/" + partName + "/name", partName );
    nh.setParam("dashboard/" + partName + "/homeable", homeable );
    nh.setParam("dashboard/" + partName + "/homingmandatory", homingmandatory );
    nh.setParam("dashboard/" + partName + "/resettable", resettable );
	
	return true;
}

void Supervisor::displaySupervisoredPeers()
{
    cout << endl;
    cout << "List of Supervisored peers : " << endl;
	// Display  AllwaysOnComponents
    cout << endl;
    cout << "AllwaysOnList : " << endl;

    vector<TaskContext*>::iterator i;
    for ( i = AllwaysOnList.begin() ; i != AllwaysOnList.end() ; i++ ) {
        TaskContext* tc = (*i);

        if( tc == NULL ) {
            cout << "AllwaysOnList contains null values ! (displaySupervisoredPeers)" << endl;
        } else {
            cout << "--  " << tc->getName() << endl;
        }
    }
    cout << "------------------------" << endl;
    
	// Display Enabled 
	cout << endl;
    for( int partNr = 1; partNr < 6; partNr++ ) {
		cout << "EnabledList[" << partNr << "] = [";

		vector<TaskContext*>::iterator i;
		for ( i = EnabledList[partNr].begin() ; i != EnabledList[partNr].end() ; i++ ) {
			TaskContext* tc = (*i);
			if( tc == NULL ) {
				cout << "EnabledList[" << partNr << "] contains null values ! (displaySupervisoredPeers)" << endl;
			} else if (i == EnabledList[partNr].begin()) {
				cout << tc->getName();
			}
			else {
				cout << ", " << tc->getName();
			}
		}
		cout << "]" << endl;
	}	  
	
	cout << "------------------------" << endl;
	cout << endl; 

	// Display HomingOnly 
	cout << endl;
    for( int partNr = 1; partNr < 6; partNr++ ) {
		cout << "HomingOnlyList[" << partNr << "] = [";

		vector<TaskContext*>::iterator i;
		for ( i = HomingOnlyList[partNr].begin() ; i != HomingOnlyList[partNr].end() ; i++ ) {
			TaskContext* tc = (*i);
			if( tc == NULL ) {
				cout << "HomingOnlyList[" << partNr << "] contains null values ! (displaySupervisoredPeers)" << endl;
			} else if (i == HomingOnlyList[partNr].begin()) {
				cout << tc->getName();
			}
			else {
				cout << ", " << tc->getName();
			}
		}
		cout << "]" << endl;
	}	  
	
	cout << "------------------------" << endl;
	cout << endl;

	// Display Operational Only 
	cout << endl;
    for( int partNr = 1; partNr < 6; partNr++ ) {
		cout << "OpOnlyList[" << partNr << "] = [";

		vector<TaskContext*>::iterator i;
		for ( i = OpOnlyList[partNr].begin() ; i != OpOnlyList[partNr].end() ; i++ ) {
			TaskContext* tc = (*i);
			if( tc == NULL ) {
				cout << "OpOnlyList[" << partNr << "] contains null values ! (displaySupervisoredPeers)" << endl;
			} else if (i == OpOnlyList[partNr].begin()) {
				cout << tc->getName();
			}
			else {
				cout << ", " << tc->getName();
			}
		}
		cout << "]" << endl;
	}	  
	
	cout << "------------------------" << endl;
	cout << endl;
}

void Supervisor::stopHook()
{
	for( int partNr = 1; partNr < 6; partNr++ ) {
		setState(partNr, StatusStalemsg);
	}
	hardwareStatusPort.write(hardwareStatusmsg);
	
	// send disabled msg
	enabled_rosport.write( rosdisabledmsg );
	
}

ORO_CREATE_COMPONENT(SUPERVISORY::Supervisor)
