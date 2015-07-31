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

#define maxN 6

using namespace std;
using namespace RTT;

namespace SUPERVISORY
{
	typedef vector<double> doubles;
	typedef vector<int> ints;
	typedef vector<bool> bools;
	
	/*! \class Supervisor
	 *  \brief Defines Orocos component for supervising hardware components
	 *
     * The Supervisor component monitors:
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
      
        //! Ports
        // In
        InputPort<std_msgs::Bool> ebutton_ports[4];
        InputPort<bool> homingfinished_port[maxN];
        InputPort<bool> error_port[maxN];
        InputPort<soem_beckhoff_drivers::EncoderMsg> serialRunningPort;
        InputPort<std_msgs::UInt8MultiArray> dashboardCmdPort;

        // Out
        OutputPort<diagnostic_msgs::DiagnosticArray> ebuttonStatusPort;
        OutputPort<diagnostic_msgs::DiagnosticArray> hardwareStatusPort;

        //! Properties
        vector<string> ebutton_order;

        //! Declaration
        // Scalars
        bool detected_error;
        bool emergency;
        bool goodToGO;
        long double aquisition_time;
        long double start_time;
        long double error_dected_time;
        int number_of_ebuttons;

        // Vectors
        bool homeableParts[maxN];
        bool idleDueToEmergencyButton[maxN];
        bool homedParts[maxN];
        bool staleParts[maxN];
        string bodyParts[maxN];
        vector<std_msgs::Bool> emergency_switches;
        bools allowedBodyparts;

        // Enum
        enum dashboard_cmd_t {HOMING_CMD = 21, START_CMD = 22, STOP_CMD = 23, RESET_CMD = 24};

        // Msgs
        std_msgs::UInt8MultiArray dashboardCmdmsg;
        diagnostic_msgs::DiagnosticStatus StatusStalemsg;
        diagnostic_msgs::DiagnosticStatus StatusOperationalmsg;
        diagnostic_msgs::DiagnosticStatus StatusIdlemsg;
        diagnostic_msgs::DiagnosticStatus StatusHomingmsg;
        diagnostic_msgs::DiagnosticStatus StatusErrormsg;
        diagnostic_msgs::DiagnosticArray hardwareStatusmsg;

        //! Atributes
        TaskContext* TrajectoryActionlib;
        Attribute<bools> AllowReadReferencesRefGen;

        //! Component lists
        vector<TaskContext*> AllwaysOnList;
        vector<TaskContext*> OpOnlyList[maxN];
        vector<TaskContext*> HomingOnlyList[maxN];
        vector<TaskContext*> EnabledList[maxN];

        //! Component Hook functions
        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();

        //! External functions
        virtual bool CreateRobotObject(string robotName, vector<string> defaultBodyParts);
        virtual bool AddBodyPart( int partNr, string partName, bool homeable , bool homingmandatory, bool resettable);
        virtual bool AddAllwaysOnPeer(string peerName);
        virtual bool AddOpOnlyPeer(string peerName, int partNr);
        virtual bool AddHomingOnlyPeer(string peerName, int partNr);
        virtual bool AddEnabledPeer(string peerName, int partNr);
        virtual bool StartBodyPart( string partName );
        virtual bool StopBodyPart( string partName );

        Supervisor(const string& name);
        virtual ~Supervisor();    

		protected:

        virtual bool AddPeerCheckList( string peerName, vector<TaskContext*> List );
        virtual bool startList( vector<TaskContext*> List );
        virtual bool stopList( vector<TaskContext*> List );
        virtual bool setState(int partNr, diagnostic_msgs::DiagnosticStatus state);
        virtual bool GoOperational(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoIdle(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoHoming(int partNr, diagnostic_msgs::DiagnosticArray statusArray);
        virtual bool GoError(int partNr, diagnostic_msgs::DiagnosticArray statusArray);


      };
}

#endif
