/*
 * Supervisor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 * Wrecked by Tim 
 */

#ifndef GENSUPERVISOR_HPP_
#define GENSUPERVISOR_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <rosgraph_msgs/Log.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>

using namespace std;
using namespace RTT;

namespace SUPERVISORY
{
    /** \ingroup ARP-arp_core
     *
     * \class GenSupervisor
     *
     * Cette classe permet de gerer plusieurs composants. Il y en a au moins une par projet
     * qui permet de faire l'interface de pilotage pour les projets suprieurs.
     *
     * Generic variant of Tim's work
     *
     */
    class GenSupervisor
    : public RTT::TaskContext
      {
      public:

        InputPort<std_msgs::Bool> rosemergencyport;
        InputPort<std_msgs::Bool> rosshutdownport;
        OutputPort<std_msgs::Bool> isenabled_rosport[9];
        OutputPort<std_msgs::Bool> enabled_rosport;
        InputPort<std_msgs::Bool> fireup_rosport[9];
        OutputPort<diagnostic_msgs::DiagnosticArray> hardwareStatusPort;

        bool enabled[9]; //Keep track if bodyparts are on or off
        bool emergency;
        bool fireup[9];
        string bodyparts[9];
		long double aquisition_time;
		long double start_time;
		
		// Port for checking Soem 
		//InputPort<soem_beckhoff_drivers::EncoderMsg> serialRunningPort;
		InputPort<bool> serialRunningPort;

        /** Constructeur pour dfinir le chemin vers le projet. Utile pour ROS*/
        GenSupervisor(const std::string& name);
        /** Destructeur par dfaut */
        virtual ~GenSupervisor();

        /**
         * Configure all peers previously registered by the AddAllwaysOnPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the AddAllwaysOnPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Stop all peers previously registered by the AddAllwaysOnPeer command
         * The configuration is done in the reverse order in which elements where inserted
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the AddAllwaysOnPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be Supervisored
         * @return true if success, false if the peer to GenSupervisor is not in the peer list.
         */
        virtual bool AddAllwaysOnPeer(std::string peerName );
        virtual bool AddFinishedHomingPeer(std::string peerName );
        virtual bool AddPeerToBodyPart( std::string peerName, int partNr );
        virtual bool NameBodyPart( int partNr, std::string partName );
        virtual bool StartBodyPart( std::string partName );
        virtual bool StopBodyPart( std::string partName );
        
        /**
         * Internal functions for convenience
         */
        virtual bool AddPeerCheckList( std::string peerName, vector<TaskContext*> List );
        virtual bool startList( vector<TaskContext*> List );
        virtual bool stopList( vector<TaskContext*> List );
        virtual bool isEmpty( vector<TaskContext*> List );

        /**
         * display the list of Supervisored peers
         */
        virtual void displaySupervisoredPeers();

        /**
         * Status messages to be sent to dashboard
         */

        //To Do: Make generic by defining this via component name
        diagnostic_msgs::DiagnosticArray hardwareStatusMsg;
        int cntr;
		
		diagnostic_msgs::DiagnosticStatus StatusStale;
        diagnostic_msgs::DiagnosticStatus StatusOperational;
        diagnostic_msgs::DiagnosticStatus StatusIdle;
        diagnostic_msgs::DiagnosticStatus StatusHoming;
        diagnostic_msgs::DiagnosticStatus StatusError;

    protected:
        /** List of peers to GenSupervisor */
        vector<TaskContext*> AllwaysOnList;
        vector<TaskContext*> FinishedHomingList;
        vector<TaskContext*> BodyPartList[9];
      };
}

#endif /* GenSupervisor_HPP_ */
