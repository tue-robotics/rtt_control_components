/*
 * EtherCATsupervisor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 * Wrecked by Tim 
 */

#ifndef EtherCATsupervisor_HPP_
#define EtherCATsupervisor_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
    /** \ingroup ARP-arp_core
     *
     * \class EtherCATsupervisor
     *
     * Cette classe permet de gerer plusieurs composants. Il y en a au moins une par projet
     * qui permet de faire l'interface de pilotage pour les projets supérieurs.
     */
    class EtherCATsupervisor
    : public RTT::TaskContext
      {
      public:

        InputPort<std_msgs::Bool> rosemergencyport;
        InputPort<std_msgs::Bool> rosstandbyport;
        InputPort<bool> serialRunningPort; // A port to check if soem is still running.
        bool started; //Keep track if components are on or off

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        EtherCATsupervisor(const std::string& name);
        /** Destructeur par défaut */
        virtual ~EtherCATsupervisor();

        /**
         * Configure all peers previously registered by the addEtherCATsupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the addEtherCATsupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Stop all peers previously registered by the addEtherCATsupervisoredPeer command
         * The configuration is done in the reverse order in which elements where inserted
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the addEtherCATsupervisoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be EtherCATsupervisored
         * @return true if success, false if the peer to EtherCATsupervisor is not in the peer list.
         */
        virtual bool addEtherCATsupervisoredPeer(std::string peerName );

        /**
         * display the list of EtherCATsupervisored peers
         */
        virtual void displayEtherCATsupervisoredPeers();

    protected:
        /** List of peers to EtherCATsupervisor */
        vector<TaskContext*> m_EtherCATsupervisoredList;



      };
}

#endif /* EtherCATsupervisor_HPP_ */
