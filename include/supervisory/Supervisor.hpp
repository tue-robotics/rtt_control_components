/*
 * Supervisor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 * Wrecked by Tim 
 */

#ifndef SUPERVISOR_HPP_
#define SUPERVISOR_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <std_msgs/UInt8MultiArray.h>
#include <rosgraph_msgs/Log.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace SUPERVISORY
{
	/*! \class Supervisor
	 *  \brief Defines Orocos component for supervising hardware components
	 *
	 * The Safety component monitors:
	 *
	 * 	* Dashboard Cmds -> Home - go to homing state
	 * 						Start - go to operational state
	 * 						Stop - go to idle state
	 * 						Reset - reset error and go to idle state
	 *
	 * 	* Emergency Button -> pressed - go to idle state
	 * 						  released - go to operational state (only
	 * 						  if idle state was entered due to emergency button)
	 *   
	 *  * Errorports -> if boolean true - go to error state
	 * 
	 *  * Homingports -> if boolean ture - go to operational state
	 */

    class Supervisor
    : public RTT::TaskContext
      {
      public:
		//ports
        InputPort<std_msgs::Bool> rosemergencyport;
        InputPort<std_msgs::Bool> rosshutdownport;
        OutputPort<std_msgs::Bool> enabled_rosport;
        InputPort<bool> homingfinished_port[6];
        InputPort<bool> error_port[6]; 
        InputPort<soem_beckhoff_drivers::EncoderMsg> serialRunningPort;
        InputPort<std_msgs::UInt8MultiArray> dashboardCmdPort;
        OutputPort<std_msgs::Bool> isenabled_rosport[6];
        OutputPort<diagnostic_msgs::DiagnosticArray> hardwareStatusPort;
        
		//vectors
        bool homeableParts[6];
        bool idleDueToEmergencyButton[6];
        bool homedParts[6];
        bool staleParts[6]; // staleparts is used to make sure, components that aren't started will be shown stale on the dashboard
        string bodyParts[6];
        
        // scalars
        bool emergency;     
        bool goodToGO;   
		long double aquisition_time;
		long double start_time;
		
		// msgs
        std_msgs::UInt8MultiArray dashboardCmdmsg;
        std_msgs::Bool rosenabledmsg;
        std_msgs::Bool rosdisabledmsg;
        diagnostic_msgs::DiagnosticStatus StatusStalemsg;
        diagnostic_msgs::DiagnosticStatus StatusOperationalmsg;
        diagnostic_msgs::DiagnosticStatus StatusIdlemsg;
        diagnostic_msgs::DiagnosticStatus StatusHomingmsg;
        diagnostic_msgs::DiagnosticStatus StatusErrormsg;
        diagnostic_msgs::DiagnosticArray hardwareStatusmsg;

        Supervisor(const std::string& name);
        virtual ~Supervisor();

		// component Hook functions
        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        
        // External functions
        virtual bool StartBodyPart( std::string partName );
        virtual bool StopBodyPart( std::string partName );
        virtual void displaySupervisoredPeers();

		// Internal functions
        virtual bool AddPeerCheckList( std::string peerName, vector<TaskContext*> List );
        virtual bool startList( vector<TaskContext*> List );
        virtual bool stopList( vector<TaskContext*> List );
        virtual bool isEmpty( vector<TaskContext*> List );

        // Set up functions for name body part and add component to list
        virtual bool NameBodyPart(int partNr, std::string partName, bool homeable);
		virtual bool AddAllwaysOnPeer(std::string peerName);
        virtual bool AddOpOnlyPeer(std::string peerName, int partNr);
        virtual bool AddHomingOnlyPeer(std::string peerName, int partNr);
        virtual bool AddEnabledPeer(std::string peerName, int partNr);
        
        // state transitions
        virtual bool GoOperational(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoIdle(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoHoming(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoError(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool setState(int partNr, diagnostic_msgs::DiagnosticStatus state);

    protected:
		// Component lists
        vector<TaskContext*> AllwaysOnList;
        vector<TaskContext*> OpOnlyList[6];
        vector<TaskContext*> HomingOnlyList[6];
        vector<TaskContext*> EnabledList[6];
        
      };
}

#endif
