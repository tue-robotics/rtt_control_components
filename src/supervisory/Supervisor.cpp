/*
 * Supervisor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 * Wrecked by Tim 
 */
//#include <ros/package.h>
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
    addOperation("NameBodyPart", &Supervisor::NameBodyPart, this, OwnThread)
            .doc("Name a body part, this also creates a topic stating this bodypart is enabled or not")
            .arg("partNr","The number of the bodypart")              
            .arg("partName","The name of the bodypart"); 
    addOperation("StartBodyPart", &Supervisor::StartBodyPart, this, OwnThread)
            .doc("Start a body part")
            .arg("partName","The name of the bodypart");              
    addOperation("StopBodyPart", &Supervisor::StopBodyPart, this, OwnThread)
            .doc("Stop a body part")
            .arg("partName","The name of the bodypart");  
    addOperation("DisplaySupervisoredPeers", &Supervisor::displaySupervisoredPeers, this, ClientThread)
            .doc("Display the list of peers");
  addPort( "rosemergency", rosemergencyport );
  addPort( "rosshutdown", rosshutdownport );
  addPort( "rosetherCATenabled", enabled_rosport );
  addPort( "serialRunning", serialRunningPort ).doc("Serial device running port");
  addPort( "dashboardCmd", dashboardCmdPort ).doc("To receive dashboard commands ");  
  addPort("hardware_status", hardwareStatusPort ).doc("To send status to dashboard ");
}

Supervisor::~Supervisor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool Supervisor::configureHook()
{
    Logger::In in("Supervisor::Configure");

	// declaration of scalars
	emergency = false;
	goodToGO = false;
	start_time = 0.0;
	aquisition_time = 0.0;

	// declaration of msgs
	rosenabledmsg.data = true;
	rosdisabledmsg.data = false;
	dashboardCmdmsg.data.assign(2, 0.0);	
	StatusStalemsg.level = 0;
	StatusIdlemsg.level = 1;
	StatusOperationalmsg.level = 2;
	StatusHomingmsg.level = 3;
	StatusErrormsg.level = 4;
	hardwareStatusmsg.status.resize(6);
	
	// declaration of  vectors
	for( int partNr = 0; partNr < 6; partNr++ ) {
		hardwareStatusmsg.status[partNr] = StatusStalemsg; 
		homeableParts[partNr] = false;
		idleDueToEmergencyButton[partNr] = false;
		homedParts[partNr] = false;
		staleParts[partNr] = true; // all parts are stale by default, in Namebodypart function they will be set to false
		bodyParts[partNr] = "";
	}
	
	staleParts[0] = false; // whole robot is never stale

	enabled_rosport.write( rosenabledmsg );
	
	return true;
}

bool Supervisor::startHook()
{
    Logger::In in("Supervisor::Start");

	bodyParts[0] = "all";
	hardwareStatusmsg.status[0].name = bodyParts[0];
	
	if ( !serialRunningPort.connected() )
	{
		log(Error) << "Supervisor: serialport to check soem is not connected" << endlog();
		return false;
	}
	
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	aquisition_time = start_time;
	//Wait untill soem is running
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
	
    return true;
}

void Supervisor::updateHook()
{
    Logger::In in("Supervisor::Update");

	// time out for dashboard calls
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
  
	//bool serialRunning = false;
	soem_beckhoff_drivers::EncoderMsg serialRunning;
	if(serialRunningPort.read(serialRunning) == NewData) 
	{
		aquisition_time = new_time;
	}
	else if ( new_time - aquisition_time > 1.0 )
	{
		ROS_ERROR_STREAM("Supervisor: Soem crashed!");
		enabled_rosport.write( rosdisabledmsg );
	} 
	
	// Check if emergency button pressed:
	std_msgs::Bool rosemergencymsg;
	if ( rosemergencyport.read( rosemergencymsg ) == NewData )
	{
		if ( emergency != rosemergencymsg.data && rosemergencymsg.data )
		{
			ROS_INFO_STREAM( "Supervisor: Emergency button pressed, shutting down components online components" );
			for ( int partNr = 1; partNr < 6; partNr++ ) {
				if ( hardwareStatusmsg.status[partNr].level == StatusOperationalmsg.level ) {
					idleDueToEmergencyButton[partNr] = true;
				}
				GoIdle(partNr,hardwareStatusmsg);
			}
			//log(Warning) << "Supervisor: idleDueToEmergencyButton: [" << idleDueToEmergencyButton[1] << "," << idleDueToEmergencyButton[2] << "," << idleDueToEmergencyButton[3] << "," << idleDueToEmergencyButton[4] << "," << idleDueToEmergencyButton[5] << "]" << endlog();  
		}
		else if ( emergency != rosemergencymsg.data && !rosemergencymsg.data )
		{
			ROS_INFO_STREAM( "Supervisor: Emergency button released, restoring components" );
			for ( int partNr = 1; partNr < 6; partNr++ ) {
				if ( ( hardwareStatusmsg.status[partNr].level == StatusIdlemsg.level ) && (idleDueToEmergencyButton[partNr] == true)) {
					GoOperational(partNr,hardwareStatusmsg);
					idleDueToEmergencyButton[partNr] = false;
				}
			}
		}
		emergency = rosemergencymsg.data;
	}

	// Read DashboardCmds
	if (goodToGO) {
		if ( dashboardCmdPort.read(dashboardCmdmsg) == NewData ) {
			if (dashboardCmdmsg.data[0] == 0) { // 0 = all bodyparts
                if (dashboardCmdmsg.data[1] == HOMING_CMD && emergency == false ) {
					log(Warning) << "Supervisor: Received Homing request from dashboard for all parts" << endlog();      
					for ( int partNr = 1; partNr < 6; partNr++ ) {
						if (homeableParts[partNr]) { 
							GoHoming(partNr,hardwareStatusmsg);
							}
						else { 
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
    
	// Check for shutdown command
	std_msgs::Bool rosshutdownmsg;
	rosshutdownport.read( rosshutdownmsg );
	if ( rosshutdownmsg.data == true )
	{
		ROS_INFO_STREAM( "Shutdown requested" );
		stop();
	}
}

void Supervisor::stopHook()
{
    Logger::In in("Supervisor::Stop");

	for( int partNr = 0; partNr < 6; partNr++ )   // Loop over the bodyparts
	{
		if ( ! isEmpty( EnabledList[partNr] ) )  // That exist
		{
			stopList( OpOnlyList[partNr] );
			stopList( EnabledList[partNr] );
			stopList( HomingOnlyList[partNr] ); 
		}
	}
	stopList( AllwaysOnList );
	enabled_rosport.write( rosdisabledmsg );
}

//-----------------------------------------------------

bool Supervisor::AddPeerCheckList( std::string peerName, vector<TaskContext*> List )
{    
    Logger::In in("Supervisor::Stop");

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

bool Supervisor::StartBodyPart( std::string partName )
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

bool Supervisor::StopBodyPart( std::string partName )
{
	for ( int partNr = 0; partNr < 6; partNr++ )
	{
		string ipartName = bodyParts[partNr];
		if (ipartName.compare(partName) == 0)
		{
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
	return true;
}

bool Supervisor::GoOperational(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
	if (staleParts[partNr] == false) {
		// if in error state, GoOp is blocked, if in homing state, the EnabledList does not need to be restarted
		if (statusArray.status[partNr].level != StatusErrormsg.level && statusArray.status[partNr].level != StatusHomingmsg.level) {
			stopList( HomingOnlyList[partNr] );
			startList( EnabledList[partNr] );
			startList( OpOnlyList[partNr] );
			setState(partNr, StatusOperationalmsg);
		}
		else if (statusArray.status[partNr].level != StatusErrormsg.level && statusArray.status[partNr].level == StatusHomingmsg.level) {
			stopList( HomingOnlyList[partNr] );
			startList( OpOnlyList[partNr] );
			setState(partNr, StatusOperationalmsg);
		}
	}
	else {
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
		if (statusArray.status[partNr].level != StatusErrormsg.level) {
			stopList( OpOnlyList[partNr] );
			startList( EnabledList[partNr] );
			startList( HomingOnlyList[partNr] );
			setState(partNr, StatusHomingmsg);
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

bool Supervisor::AddAllwaysOnPeer(std::string peerName )
{
	if ( AddPeerCheckList( peerName, AllwaysOnList ) )
	{
		AllwaysOnList.push_back ( getPeer(peerName) ); 
		getPeer(peerName)->start();
		return true;
	}
	return false;
}

bool Supervisor::AddOpOnlyPeer( std::string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, OpOnlyList[partNr] ) )
	{
		OpOnlyList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::AddHomingOnlyPeer( std::string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, HomingOnlyList[partNr] ) )
	{
		HomingOnlyList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::AddEnabledPeer( std::string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, EnabledList[partNr] ) )
	{
		EnabledList[partNr].push_back ( getPeer(peerName) ); 
		return true;
	}
	return false;
}

bool Supervisor::NameBodyPart( int partNr, std::string partName, bool homeable )
{
    addPort( partName+"_homingfinished", homingfinished_port[partNr] );
    addPort( partName+"_error", error_port[partNr] );
	bodyParts[partNr] = partName;
	homeableParts[partNr] = homeable;
	staleParts[partNr] = false;
	return true;
}

void Supervisor::displaySupervisoredPeers()
{
    cout << endl;
    cout << "List of Supervisored peers : " << endl;
    cout << endl;
    cout << "AllwaysOnList : " << endl;

    vector<TaskContext*>::iterator i;
    for ( i = AllwaysOnList.begin() ; i != AllwaysOnList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "AllwaysOnList contains null values ! (displaySupervisoredPeers)" << endl;
        }
        else
        {
            cout << "--  " << tc->getName() << endl;
        }
    }

    cout << "------------------------" << endl;
    cout << endl;
    for( int partNr = 0; partNr < 6; partNr++ )   
    {

		cout << "EnabledList[" << partNr << "] : " << endl;

		vector<TaskContext*>::iterator i;
		for ( i = EnabledList[partNr].begin() ; i != EnabledList[partNr].end() ; i++ )
		{
			TaskContext* tc = (*i);

			if( tc == NULL )
			{
				cout << "EnabledList[" << partNr << "] contains null values ! (displaySupervisoredPeers)" << endl;
			}
			else
			{
				cout << "--  " << tc->getName() << endl;
			}
		}

		cout << "------------------------" << endl;
		cout << endl; 
	}  
}


ORO_CREATE_COMPONENT(SUPERVISORY::Supervisor)
