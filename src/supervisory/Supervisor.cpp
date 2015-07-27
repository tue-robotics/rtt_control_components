#include <rtt/Component.hpp>
#include "Supervisor.hpp"
#include <ros/ros.h>

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

Supervisor::Supervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    //! Operations
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
    addOperation("StartBodyPart", &Supervisor::StartBodyPart, this, OwnThread)
            .doc("Start a body part")
            .arg("partName","The name of the bodypart");              
    addOperation("StopBodyPart", &Supervisor::StopBodyPart, this, OwnThread)
            .doc("Stop a body part")
            .arg("partName","The name of the bodypart");
    
    //! Properties
	addProperty( "ebuttonorder", ebutton_order );

    //! Ports
    // In
    addPort( "serialRunning", serialRunningPort ).doc("Serial device running port");
    addPort( "dashboardCmd", dashboardCmdPort ).doc("To receive dashboard commands ");

    // Out
    addPort( "hardware_status", hardwareStatusPort ).doc("To send hardware status to dashboard");
    addPort( "ebutton_status", ebuttonStatusPort ).doc("To send ebutton status to dashboard");
}

Supervisor::~Supervisor()
{
	//! Set TaskContext pointers to NULL
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

    //! remove dashboard ros parameters from parameter server
    ros::NodeHandle nh("~");
    nh.deleteParam("dashboard");
    nh.deleteParam("dashboard_list");
}

//! component Hook functions
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

    if ( this->hasPeer( "TrajectoryActionlib") ) {
        TrajectoryActionlib = this->getPeer( "TrajectoryActionlib");
        AllowReadReferencesRefGen = TrajectoryActionlib->attributes()->getAttribute("allowedBodyparts");
    }

    // Check Property Acces
    if (!AllowReadReferencesRefGen.ready() ) {
        log(Error) << "Supervisor: Could not gain acces to GlobalReferenceGenerator.AllowReadReferences"<<endlog();
        return false;
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

    // Listen to error ports
	for ( int partNr = 0; partNr < 6; partNr++ ) {
		bool error;
		if ( error_port[partNr].read( error ) == NewData ) {
			if (error == true) {
				GoError(partNr,hardwareStatusmsg);
			}
		}
	}

    // Send hardware status to dashboard
	if (goodToGO) {
		hardwareStatusPort.write(hardwareStatusmsg);
	}

}

void Supervisor::stopHook()
{
    for( int partNr = 1; partNr < 6; partNr++ ) {
        setState(partNr, StatusStalemsg);
    }
    hardwareStatusPort.write(hardwareStatusmsg);
}

//! Inititialization functions
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

//! List operations
bool Supervisor::startList( vector<TaskContext*> List )
{
    vector<TaskContext*>::iterator i;
    for ( i = List.begin() ; i != List.end() ; i++ ) {
        TaskContext* tc = (*i);
        string componentname = tc->getName();

        if (!tc->isRunning() ) {
            tc->start();
        }

        if ( !tc->isRunning() ) {
            log(Error) << "Supervisor::startList: " + componentname + " could not be started, trying again." << endlog();
            tc->start();

            if ( !tc->isRunning() ) {
                log(Error) << "Supervisor::startList: " + componentname + " could not be started." << endlog();
                return false;
            }
        }
    }
    return true;
}

bool Supervisor::stopList( vector<TaskContext*> List )
{
    vector<TaskContext*>::iterator i;
    for ( i = List.begin() ; i != List.end() ; i++ ) {
        TaskContext* tc = (*i);
        string componentname = tc->getName();
        tc->stop();
    }

    return true;
}

//! Bodypart operations
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
    // Set status
    hardwareStatusmsg.status[partNr] = state;

    // Add homed status and partname
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

bool Supervisor::GoOperational(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
    if (staleParts[partNr] == false) {
        if (statusArray.status[partNr].level != StatusErrormsg.level) {				// if in error state, GoOp is blocked

            // Start and stop lists
            stopList( HomingOnlyList[partNr] );
            startList( OpOnlyList[partNr] );
            if (statusArray.status[partNr].level != StatusHomingmsg.level) {		// if in homing state, the EnabledList does not need to be restarted
                startList( EnabledList[partNr] );
            }

            // Allow reading of references
            allowedBodyparts = AllowReadReferencesRefGen.get();
            allowedBodyparts[partNr-1] = true;
            AllowReadReferencesRefGen.set(allowedBodyparts);

            // Set state
            setState(partNr, StatusOperationalmsg);

        } else {
            log(Error) << "Supervisor::GoOperational: Could not go into Operational state, since this part is in Error" << endlog();
        }
    } else {
        setState(partNr, StatusStalemsg);
        log(Error) << "Supervisor::GoOperational: Could not go into Operational state, since this part is stale" << endlog();
    }

    return true;
}

bool Supervisor::GoIdle(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
    if (staleParts[partNr] == false) {

        // Stop all lists
        stopList( EnabledList[partNr] );
        stopList( HomingOnlyList[partNr] );
        stopList( OpOnlyList[partNr] );

        // Disallow bodypart to obtain references
        allowedBodyparts = AllowReadReferencesRefGen.get();
        allowedBodyparts[partNr-1] = false;
        AllowReadReferencesRefGen.set(allowedBodyparts);

        // Set state
        setState(partNr, StatusIdlemsg);
    }
    else {
        setState(partNr, StatusStalemsg);
        log(Error) << "Supervisor::GoIdle: Could not go into Idle state, since this part is stale" << endlog();
    }

    return true;
}

bool Supervisor::GoHoming(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
    if (staleParts[partNr] == false) {
        if (statusArray.status[partNr].level != StatusErrormsg.level) {

            // Start and stop lists
            stopList( OpOnlyList[partNr] );
            startList( EnabledList[partNr] );
            startList( HomingOnlyList[partNr] );

            // Disallow bodypart to obtain references
            allowedBodyparts = AllowReadReferencesRefGen.get();
            allowedBodyparts[partNr-1] = false;
            AllowReadReferencesRefGen.set(allowedBodyparts);

            // Set state
            setState(partNr, StatusHomingmsg);
        }
    }
    else {
        setState(partNr, StatusStalemsg);
        log(Error) << "Supervisor::GoHoming: Could not go into Homing state, since this part is stale" << endlog();
    }

    return true;
}

bool Supervisor::GoError(int partNr, diagnostic_msgs::DiagnosticArray statusArray)
{
    if (staleParts[partNr] == false) {
        // Stop all Lists
        stopList( EnabledList[partNr] );
        stopList( HomingOnlyList[partNr] );
        stopList( OpOnlyList[partNr] );

        // Disallow bodypart to obtain references
        allowedBodyparts = AllowReadReferencesRefGen.get();
        allowedBodyparts[partNr-1] = false;
        AllowReadReferencesRefGen.set(allowedBodyparts);

        setState(partNr, StatusErrormsg);
    }
    else {
        setState(partNr, StatusStalemsg);
        log(Error) << "Supervisor::GoError: Could not go into Error state, since this part is stale" << endlog();
    }
    return true;
}

ORO_CREATE_COMPONENT(SUPERVISORY::Supervisor)
