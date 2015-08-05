/***************************************************************************
 tag: Ton Peters  Oktober 2014  DynamixelController.hpp
 tag: Janno Lunenburg  Fri August 13 16:00:00 CET 2013  PanTiltController.hpp

 PanTiltController.hpp -  description
 -------------------
 begin                : Sat February 19 2011
 copyright            : (C) 2011 Sava Marinkov
 email                : s.marinkov@student.tue.nl

 ***************************************************************************/

#ifndef DYNAMIXELCONTROLLER_HPP
#define DYNAMIXELCONTROLLER_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <soem_beckhoff_drivers/CommMsgBig.h>
#include <sensor_msgs/JointState.h>
#include "AX12_control_table.h"
#include <queue>

#define RAD_TO_STEP             195.37860814
#define TRIAL_MAX 				20

using namespace std;
using namespace RTT;
using namespace soem_beckhoff_drivers;

namespace DYNAMIXELCONTROLLER {

    /**
       * @brief A Component that controls multiple dynamixels
       *
       * The component has one input port that should receive scalar.
       *
       * @param * fp [100 Hz] - pole frequency of the filter
       *        * dp [0.1] - pole damping
       */

    typedef std::vector<double> doubles;
    typedef std::vector<int> ints;

    class DynamixelController : public RTT::TaskContext
    {
    private:

        /* Declaring input and output ports*/
        // serial communication
        OutputPort<CommMsgBig> instructionPort;
        InputPort<CommMsgBig> statusPort;
        InputPort<bool> serialRunningPort;
        InputPort<bool> serialReadyRxPort;
        // Orocos/ROS communication
        InputPort<sensor_msgs::JointState> goalPosPort;
        OutputPort<sensor_msgs::JointState> currentPosPort;
        OutputPort<bool> errortosupervisorPort;
        InputPort<bool> enablerPort;

        /* Declaring messages*/
        CommMsgBig instruction;
        CommMsgBig status;
        //sensor_msgs::JointState goalPos; local declared for variable input
        sensor_msgs::JointState currentPos;

        /* Declaring variables set by properties*/
        ints dynamixel_ids, dynamixel_max, dynamixel_min, dynamixel_speed_default, dynamixel_offset;

        /* Declaring global variables*/
        int trial, newPosition;
        bool enable;
        int state, pstate;
        int commStatus;
        unsigned char gbInstructionPacket[MAXNUM_TXPARAM+10];
        unsigned char gbStatusPacket[MAXNUM_TXPARAM+10];
        int gbStatusSize;
        ints dynamixel_goal, dynamixel_speed;
        int read_dynamixel, n_dynamixels;

    public:
        DynamixelController(const std::string& name);
        ~DynamixelController(){};

        bool configureHook();

        bool startHook();

        void updateHook();

        void stopHook(){};

    private:
        bool readReference();

        void create_pos_goal_packet(void);
        void dxl_set_txpacket_instruction(int instruction);
        void dxl_tx_rx_packet(void);
        void dxl_set_txpacket_parameter(int index, int value);
        void dxl_set_txpacket_length(int length);

        void dxl_rx_packet(void);
        int dxl_get_rxpacket_error(int errbit);
        int dxl_rxpacket_isError(void);
        int dxl_get_rxpacket_length(void);
        int dxl_get_rxpacket_id(void);
        int dxl_get_rxpacket_parameter(int index);
        bool dxl_check_rxpacket_checksum(void);

        int dxl_makeword(int lowbyte, int highbyte);
        int dxl_get_lowbyte(int word);
        int dxl_get_highbyte(int word);

        void dxl_ping(int id);
        void dxl_read_byte(int id, int address);
        void dxl_write_byte(int id, int address, int value);
        void dxl_read_word(int id, int address);
        void dxl_write_word(int id, int address, int value);

        void dxl_tx_packet(void);
        void dxl_set_txpacket_id( int id );
        void printErrorCode(void);

    };
}
#endif
