/*
 * EtherCATsupervisor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 * Wrecked by Tim 
 */
//#include <ros/package.h>
#include <rtt/Component.hpp>
#include "EtherCATsupervisor.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;


//ORO_LIST_COMPONENT_TYPE( ETHARCAT::EtherCATsupervisor )

EtherCATsupervisor::EtherCATsupervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("AddEtherCATsupervisoredPeer", &EtherCATsupervisor::addEtherCATsupervisoredPeer, this, OwnThread)
            .doc("Add a peer to the EtherCATsupervisored list, all these components are restarted if a reset command is received!")
            .arg("peerName","Name of the peer to add to the list");

    addOperation("DisplayEtherCATsupervisoredPeers", &EtherCATsupervisor::displayEtherCATsupervisoredPeers, this, ClientThread)
               .doc("Display the list of peers");
  addEventPort( "rosemergency", rosemergencyport );
  addEventPort( "rosstandby", rosstandbyport );
  addPort("serialRunning", serialRunningPort).doc("Serial device running port");


}

EtherCATsupervisor::~EtherCATsupervisor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool EtherCATsupervisor::configureHook()
{
    return true;
}

bool EtherCATsupervisor::startHook()
{
	started = true;
    return true;
}

void EtherCATsupervisor::updateHook()
{
	// Check if soem is running
	bool serialRunning = false;
	if(!(serialRunningPort.read(serialRunning) == NewData)) 
	{
		ROS_ERROR("Soem crashed!");
	} 
	
	
    std_msgs::Bool rosemergencymsg;
    std_msgs::Bool rosstandbymsg;
    rosemergencyport.read( rosemergencymsg );
    rosstandbyport.read( rosstandbymsg );
    bool emergency = rosemergencymsg.data;
    bool standby = rosstandbymsg.data;
    
    if ( ( emergency || standby ) && started )
    {
      log(Warning) << "Stopping components" << endlog();
      vector<TaskContext*>::iterator i;
      for ( i = m_EtherCATsupervisoredList.begin() ; i != m_EtherCATsupervisoredList.end() ; i++ )
      {
          TaskContext* tc = (*i);

          if( tc == NULL )
          {
              log(Error) << "m_EtherCATsupervisoredList should not contain null values ! (update)" << endlog();
              error();
          }
          else
          {
			  log(Warning) << "EtherCATsupervisor: Stopping: " << tc->getName() << endlog();
              tc->stop();
          }
      }
      started = false;
    }  
    else if ( !( emergency || standby ) && !started )
    {
      log(Warning) << "Starting components" << endlog();
      vector<TaskContext*>::iterator i;
      for ( i = m_EtherCATsupervisoredList.begin() ; i != m_EtherCATsupervisoredList.end() ; i++ )
      {
          TaskContext* tc = (*i);

          if( tc == NULL )
          {
              log(Error) << "m_EtherCATsupervisoredList should not contain null values ! (update)" << endlog();
              error();
          }
          else
          {
			  log(Warning) << "EtherCATsupervisor: Starting: " << tc->getName() << endlog();
              tc->start();
          }
      }
      started = true;
    }
      
}

void EtherCATsupervisor::stopHook()
{
  
}

void EtherCATsupervisor::cleanupHook()
{

}


//-----------------------------------------------------

bool EtherCATsupervisor::addEtherCATsupervisoredPeer(std::string peerName )
{
    bool res = true;

    if( ! hasPeer(peerName) )
    {
        log(Error) << "You can't EtherCATsupervisor a component that is not your peer !" << endlog();
        res = false;
    }
    /*else if( getTaskState() !=  getPeer(peerName)->getTaskState() )
    {
        log(Error) << "You can't add a new EtherCATsupervisored component that is not in your current state !" << endlog();
        res = false;
    }*/
    else
    {
        vector<TaskContext*>::iterator i;
        for ( i = m_EtherCATsupervisoredList.begin() ; i != m_EtherCATsupervisoredList.end() ; i++ )
        {
            TaskContext* tc = (*i);
            if( tc == NULL )
            {
              log(Error) << "m_EtherCATsupervisoredList should not contain null values ! (addEtherCATsupervisoredPeer)" << endlog();
              res = false;
            }
            else
            {
              if( tc->getName() == peerName )
              {
                  log(Error) << tc->getName() << " is already in the list !" << endlog();
                  res = false;
                  break;
              }
            }
        }

        if( res == true )
        {
            log(Info) << "New peer to EtherCATsupervisor : " << peerName << endlog();
            m_EtherCATsupervisoredList.push_back (getPeer(peerName));
        }
    }

    return res;
}


void EtherCATsupervisor::displayEtherCATsupervisoredPeers()
{
    cout << endl;
    cout << "List of EtherCATsupervisored peers : " << endl;
    cout << endl;

    vector<TaskContext*>::iterator i;
    for ( i = m_EtherCATsupervisoredList.begin() ; i != m_EtherCATsupervisoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_EtherCATsupervisoredList should not contain null values ! (displayEtherCATsupervisoredPeers)" << endl;
        }
        else
        {
            cout << tc->getName() << endl;
        }
    }

    cout << "------------------------" << endl;
    cout << endl;
}

ORO_CREATE_COMPONENT(AMIGO::EtherCATsupervisor)
