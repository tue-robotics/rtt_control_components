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

using namespace std;
using namespace RTT;
using namespace SUPERVISORY;

Supervisor::Supervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("AddAllwaysOnPeer", &Supervisor::AddAllwaysOnPeer, this, OwnThread)
            .doc("Add a peer to the AllwaysOn list, all these components are started!")
            .arg("peerName","Name of the peer to add to the list");
    addOperation("AddPeerToBodyPart", &Supervisor::AddPeerToBodyPart, this, OwnThread)
            .doc("Add a peer to a bodypart list, all these components are stopped and if a restarted upon a fireup command!")
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
  addEventPort( "rosemergency", rosemergencyport );
  addEventPort( "rosshutdown", rosshutdownport );
  addPort( "rosetherCATenabled", enabled_rosport );
  addPort( "serialRunning", serialRunningPort ).doc("Serial device running port");

}

Supervisor::~Supervisor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool Supervisor::configureHook()
{
    return true;
}

bool Supervisor::startHook()
{
	for( int partNr = 0; partNr < 9; partNr++ ) 
	{
		fireup[partNr] = true; // Disable later
		enabled[partNr] = false;
	}
	std_msgs::Bool rosenabledmsg;
	rosenabledmsg.data = true;
	enabled_rosport.write( rosenabledmsg );
	emergency = false;
	
	if ( !serialRunningPort.connected() )
	{
		log(Error) << "Supervisor: serialport to check soem is not connected" << endlog();
		return false;
	}
	
	start_time = os::TimeService::Instance()->getNSecs()*1e-9;
	aquisition_time = start_time;
	//Wait untill soem is running
	//bool serialRunning = false;
	soem_beckhoff_drivers::EncoderMsg serialRunning;
	while (!(serialRunningPort.read(serialRunning) == NewData) )
	{
		start_time = os::TimeService::Instance()->getNSecs()*1e-9;
		if ( start_time - aquisition_time > 1.0 )
		{
			log(Error) << "No soem heartbeat found" << endlog();
			return false;
		}
	}
	aquisition_time = os::TimeService::Instance()->getNSecs()*1e-9;
		
    return true;
}

void Supervisor::updateHook()
{
	// Check if soem is running
	
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
		ROS_ERROR_STREAM("Soem crashed!");
		std_msgs::Bool rosenabledmsg;
		rosenabledmsg.data = false;
		enabled_rosport.write( rosenabledmsg );
	} 
	
	// Check if emergency button pressed:
	std_msgs::Bool rosemergencymsg;
	string startmessage = "";
	if ( rosemergencyport.read( rosemergencymsg ) == NewData )
	{
		if ( emergency != rosemergencymsg.data && rosemergencymsg.data )
		{
			ROS_INFO_STREAM( "Emergency button pressed, shutting down components online components" );
			startmessage = "Emergency button pressed: ";
		}
		else if ( emergency != rosemergencymsg.data && !rosemergencymsg.data )
		{
			ROS_INFO_STREAM( "Emergency button released, restoring components" );
			startmessage = "Emergency button released: ";
		}
		emergency = rosemergencymsg.data;
	}
			
			
	for( int partNr = 0; partNr < 9; partNr++ )    // Loop over the bodyparts
	{
		if ( ! isEmpty( BodyPartList[partNr] ) )  // That exist
		{
			// Read if a fireup is requested
			std_msgs::Bool rosfireupmsg; 
			if ( fireup_rosport[partNr].read( rosfireupmsg ) == NewData )
			{
				if ( fireup[partNr] != rosfireupmsg.data && fireup[partNr] )
				{
					//enabled[partNr] = startList( BodyPartList[partNr] );
					//log(Warning) << "Fired up " << bodyparts[partNr] <<" components" << endlog();
					ROS_INFO_STREAM( "Fireup command received for " + bodyparts[partNr] + " components" );
				}
				else if ( fireup[partNr] != rosfireupmsg.data && !fireup[partNr] )
				{
					ROS_INFO_STREAM( "Shutdown command received for " + bodyparts[partNr] + " components" );
				}						
				fireup[partNr] = rosfireupmsg.data;
			}
			
			if ( fireup[partNr] && (!enabled[partNr]) && (!emergency) ) // If no emergcy present, bodypart is down and fireup is requested:
			{
				ROS_INFO_STREAM( startmessage + "Fired up " + bodyparts[partNr] + " components" );
				if ( startList( BodyPartList[partNr] ) )
				{
					enabled[partNr] = true;
				}
				else
				{
					ROS_ERROR_STREAM( "Supervisor: " + bodyparts[partNr] + " could not be enabled, giving up");
					// Bodypart could not be enabled, stopping bodypart
					enabled[partNr] = false;
					fireup[partNr] = false;
				}

			}
			else if ( (emergency || (!fireup[partNr])) && enabled[partNr] ) // If emergency or firedown request, shut down if bodypart is up
			{ 
				stopList( BodyPartList[partNr] );
				ROS_INFO_STREAM( startmessage + "Halted " + bodyparts[partNr] + " components" );
				enabled[partNr] = false;
			}
			std_msgs::Bool rosenabledmsg;
			rosenabledmsg.data = enabled[partNr];
			isenabled_rosport[partNr].write( rosenabledmsg );
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

void Supervisor::stopHook()
{
	for( int partNr = 0; partNr < 9; partNr++ )    // Loop over the bodyparts
	{
		if ( ! isEmpty( BodyPartList[partNr] ) )  // That exist
		{
			stopList( BodyPartList[partNr] );
		}
	}
	stopList( AllwaysOnList );
	std_msgs::Bool rosenabledmsg;
	rosenabledmsg.data = false;
	enabled_rosport.write( rosenabledmsg );
}

void Supervisor::cleanupHook()
{

}


//-----------------------------------------------------

bool Supervisor::AddPeerCheckList( std::string peerName, vector<TaskContext*> List )
{
    bool res = true;
    
    if( ! hasPeer(peerName) )
    {
        log(Error) << "Supervisor: " << peerName <<": You can't Supervisor a component that is not your peer !" << endlog();
        return false;
    }
    if( ! getPeer(peerName)->isConfigured() )
    {
        ROS_ERROR_STREAM( "Supervisor: the component " + peerName + " is not yet configured. Therefore I cannot start it. Make you wonder, why would it not be configured?" );
        res = true; // I do not return false here because I want it added anyway. Than the normal process of escalation can continue
    }
    

	vector<TaskContext*>::iterator i;
	for ( i = List.begin() ; i != List.end() ; i++ )
	{
		TaskContext* tc = (*i);
		if( tc == NULL )
		{
		  log(Error) << "Supervisor: List should not contain null values !" << endlog();
		  res = false;
		}
		else
		{
		  if( tc->getName() == peerName )
		  {
			  log(Error) << tc->getName() << "Supervisor: " << peerName <<":  is already in the list !" << endlog();
			  res = false;
			  break;
		  }
		}
	}

	if( res == true )
	{
		log(Info) << "Supervisor: " << peerName <<": Peer can be succesfully added" << endlog();
	}
    return res;
}

bool Supervisor::startList( vector<TaskContext*> List )
{
	bool result = true;
	if( isEmpty( List ) )
	{
		log(Error) << "Supervisor: startList: List should not contain null values ! (update)" << endlog();
		error();
	}
	ROS_INFO_STREAM("Supervisor: starting startlist");
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
				result = false;
			}
		}
	}
	return result;
}

bool Supervisor::stopList( vector<TaskContext*> List )
{
	if( isEmpty( List ) )
	{
		log(Error) << "Supervisor: stopList: List should not contain null values ! (update)" << endlog();
		error();
	}
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

bool Supervisor::AddPeerToBodyPart( std::string peerName, int partNr )
{
	if ( AddPeerCheckList( peerName, BodyPartList[partNr] ) )
	{
		BodyPartList[partNr].push_back ( getPeer(peerName) ); 
		enabled[partNr] = false; // Reset enabled status
		return true;
	}
	return false;
}

bool Supervisor::NameBodyPart( int partNr, std::string partName, bool homeable )
{
	addPort( partName+"_enabled", isenabled_rosport[partNr] );
	addEventPort( partName+"_fireup", fireup_rosport[partNr] );
	bodyparts[partNr] = partName;
	homeableParts[partNr] = homeable;
	return true;
}


bool Supervisor::StartBodyPart( std::string partName )
{
	for ( int partNr = 0; partNr < 9; partNr++ )
	{
		string ipartName = bodyparts[partNr];
		if (ipartName.compare(partName) == 0)
		{
			// startList( BodyPartList[partNr] ); // This goes wrong if emergency button is pressed
			fireup[partNr] = true;
			return true;
		}
	}
	log(Error) << "Supervisor: StartBodyPart: No such bodypart" << endlog();
	return false;
}
	
	
bool Supervisor::StopBodyPart( std::string partName )
{
	for ( int partNr = 0; partNr < 9; partNr++ )
	{
		string ipartName = bodyparts[partNr];
		if (ipartName.compare(partName) == 0)
		{
			fireup[partNr] = false;
			stopList( BodyPartList[partNr] );
			return true;
		}
	}
	log(Error) << "Supervisor: StartBodyPart: No such bodypart" << endlog();
	return false;
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
    for( int partNr = 0; partNr < 9; partNr++ )   
    {

		cout << "BodyPartList[" << partNr << "] : " << endl;

		vector<TaskContext*>::iterator i;
		for ( i = BodyPartList[partNr].begin() ; i != BodyPartList[partNr].end() ; i++ )
		{
			TaskContext* tc = (*i);

			if( tc == NULL )
			{
				cout << "BodyPartList[" << partNr << "] contains null values ! (displaySupervisoredPeers)" << endl;
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
