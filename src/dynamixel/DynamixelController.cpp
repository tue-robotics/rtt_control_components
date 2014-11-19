/***************************************************************************
 tag: Ton Peters  Oktober 2014  DynamixelController.hpp
 tag: Janno Lunenburg  Fri August 13 16:00:00 CET 2013  PanTiltControllerJointState.hpp

 PanTiltController.cpp -  description
 -------------------
 begin                : Sat February 19 2011
 copyright            : (C) 2011 Sava Marinkov
 email                : s.marinkov@student.tue.nl

 ***************************************************************************/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include "DynamixelController.hpp"

using namespace std;
using namespace RTT;
using namespace soem_beckhoff_drivers;
using namespace DYNAMIXEL;

DynamixelController::DynamixelController(const std::string& name) :
	TaskContext(name, PreOperational) {
	addPort("instruction", instructionPort).doc("Dynamixel instruction packet port");
	addPort("status", statusPort).doc("Dynamixel status packet port");
	addPort("serialRunning", serialRunningPort).doc("Serial device running port");
	addPort("serialReadyRx", serialReadyRxPort).doc("Serial device ready receive port");	
	addPort("goalPos", goalPosPort).doc("Goal head position");
	addPort("currentPos", currentPosPort).doc("Current head position");
	addPort( "errortosupervisor", errortosupervisorPort );
	addPort( "enabler", enablerPort ); 

	addProperty( "dynamixel_ids", dynamixel_ids).doc("dynamixel dynamixel id");
	addProperty( "dynamixel_max", dynamixel_max).doc("dynamixel max angle, 0 to 1023");
	addProperty( "dynamixel_min", dynamixel_min).doc("dynamixel min angle, 0 to 1023");
	addProperty( "dynamixel_speed", dynamixel_speed_default).doc("dynamixel speed, 0 to 1023");
	addProperty( "dynamixel_offset", dynamixel_offset).doc("dynamixel offset angle, 0 to 1023");
	addProperty( "joint_names", currentPos.name).doc("Array containing strings withjoint names");
	// ToDO: Add status port, check http://www.google.com/url?q=http://www.electronickits.com/robot/BioloidAX-12(english).pdf&sa=U&ei=xYA1T5r6IYLBhAfHlZnqAQ&ved=0CAQQFjAA&client=internal-uds-cse&usg=AFQjCNFq1o8ghhtWNHEBkxUA5kbnz17hpw
	log(Debug) << "DynamixelController constructor done." << endlog();
}

bool DynamixelController::configureHook() 
{
	// Get the number of dynamixels
	n_dynamixels = dynamixel_ids.size();

	// Check if the properties are set correctly
	if ( n_dynamixels < 1 || n_dynamixels > 253 ){
		log(Error)<<"DynamixelController: propertie dynamixel_ids invallid!" << endlog();
		return false;
	}
	if ( dynamixel_max.size() != n_dynamixels ||
		 dynamixel_min.size() != n_dynamixels ||
		 dynamixel_speed_default.size() != n_dynamixels ||
		 dynamixel_offset.size() != n_dynamixels ){
		log(Error) << "DynamixelController: size of max, min, speed and offset should be equal to size of ids!" << endlog();
		return false;
	}
	for ( uint i=0; i<n_dynamixels; i++ ){
		if ( dynamixel_ids[i] < 0 || dynamixel_ids[i] > 253 ){
			log(Error)<< "DynamixelController: Id "<<dynamixel_ids[i]<<", property dynamixel_id invallid (range 0~253)!" << endlog();
			return false;
		}
		if ( dynamixel_max[i] < 0 || dynamixel_max[i] > 1023 ||
			 dynamixel_min[i] < 0 || dynamixel_min[i] > 1023 ||
			 dynamixel_speed_default[i] < 0 || dynamixel_speed_default[i] > 1023 ||
			 dynamixel_offset[i] < 0 || dynamixel_offset[i] > 1023 ){
			log(Error) << "DynamixelController: Id "<<dynamixel_ids[i]<<", properties max, min, speed and offset should be in range 0~1023!" << endlog();
			return false;
		}
		if ( dynamixel_max[i] < dynamixel_min[i] )
			log(Warning)<< "DynamixelController: Id "<<dynamixel_ids[i]<<", property dynamixel_max < dynamixel_min ???" << endlog();
	}

	// Initialize variables
	state = 1;
	for (int i = 0; i < MAXNUM_TXPARAM+10; i++) {
		gbInstructionPacket[i] = 0;
		gbStatusPacket[i] = 0;
	}
	currentPos.position.assign(n_dynamixels, 0.0);
	newPosition = 0;
	commStatus = COMM_RXSUCCESS;
	enable = false;
	dynamixel_speed.assign(n_dynamixels,0);
	dynamixel_goal.assign(n_dynamixels,0);
	
	log(Debug) << "DynamixelController configuration done." << endlog();
	return true;
}

bool DynamixelController::startHook() 
{
	for ( uint i=0; i<n_dynamixels; i++ ){
		log(Debug) << "dynamixel_ids " << dynamixel_ids[i] << endlog();
		log(Debug) << "dynamixel_max " << dynamixel_max[i] << endlog();
		log(Debug) << "dynamixel_min " << dynamixel_min[i] << endlog();
		log(Debug) << "dynamixel_speed " << dynamixel_speed_default[i] << endlog();
		log(Debug) << "dynamixel_offset " << dynamixel_offset[i] << endlog();
	}
	log(Debug) << "DynamixelController start done." << endlog();
	return true;
}

bool DynamixelController::readReference() 
{
    sensor_msgs::JointState goalPos;
	if (goalPosPort.read(goalPos) == NewData) {
		log(Debug) << "DynamixelController: new reference obtained."<< endlog();
        uint n_inputs = goalPos.name.size();
        // loop over inputs
        for ( uint j=0; j<n_inputs; j++ ){
            // find the corresponding dynamixel and set command
            for ( uint i=0; i<n_dynamixels; i++ ){
                if (goalPos.name[j] == currentPos.name[i] ) {
                    // found matching dynamixel
                    dynamixel_goal[i] = (int) ((goalPos.position[j])*RAD_TO_STEP+dynamixel_offset[i]);
                    if (dynamixel_goal[i] > dynamixel_max[i])
                        dynamixel_goal[i] = dynamixel_max[i];
                    if (dynamixel_goal[i] < dynamixel_min[i])
                        dynamixel_goal[i] = dynamixel_min[i];
                    if (goalPos.velocity.size() > 0) {
                        dynamixel_speed[i] = (int) ((goalPos.velocity[j])*RAD_TO_STEP);
                        log(Debug) << "Incoming dynamixel speed: " << dynamixel_speed[i] <<endlog();
                        if (dynamixel_speed[i] > 1023)
                            dynamixel_speed[i] = 1023;
                        if (dynamixel_speed[i] == 0)
                            dynamixel_speed[i] = dynamixel_speed_default[i];
                    }
                    else {
                        dynamixel_speed[i] = dynamixel_speed_default[i];
                    }
                    break;
                } else {
                    if ( i >= n_dynamixels-1 ){
                        //log(Debug)<<"DynamixelController: JointState message should contain "<<currentPos.name[i]<<", while this is now "<<goalPos.name[i] <<endlog();
                        log(Debug)<<"DynamixelController: JointState message should contain the name of a dynamixel, while this is now "<<goalPos.name[i] <<endlog();
                    }
                }
            }
        }
		return true;
	}
	return false;
}

void DynamixelController::updateHook() 
{
	enablerPort.read(enable);
	if (enable == false) {
		currentPos.header.stamp = ros::Time::now();
		currentPosPort.write(currentPos);
		return;
	}
	
	bool serialRunning = false;
	if(!(serialRunningPort.read(serialRunning) == NewData)) {
		return;
	}
	currentPos.header.stamp = ros::Time::now();
	if (commStatus == COMM_RXSUCCESS){
		trial = 0;
		pstate = state;
		switch (state) {
			case 1:
				log(Debug) << "DynamixelController: setting speeds to default and position to offset" << endlog();
				for ( uint i=0; i<n_dynamixels; i++ ){
					dynamixel_speed[i] = dynamixel_speed_default[i];
					dynamixel_goal[i] = dynamixel_offset[i];
				}
				create_pos_goal_packet();
				dxl_tx_rx_packet();
				read_dynamixel = 0;
				state ++;
				break;
			case 2:
				log(Debug) << "DynamixelController: Requesting current dynamixel position"<< endlog();
				dxl_read_word(dynamixel_ids[read_dynamixel], AX_PRESENT_POSITION_L);
				state++;
				break;
			case 3:
				newPosition = dxl_makeword(dxl_get_rxpacket_parameter(0), dxl_get_rxpacket_parameter(1));
				log(Debug) << "DynamixelController: dynamixel id, " << read_dynamixel+1 << ", received new position, " << newPosition << endlog();
				currentPos.position[read_dynamixel] = (newPosition-dynamixel_offset[read_dynamixel])/RAD_TO_STEP;
                currentPosPort.write(currentPos);
				read_dynamixel ++;
				if ( read_dynamixel < n_dynamixels ){
					log(Debug) << "DynamixelController: Read position of dynamixel, id " << read_dynamixel+1 << endlog();
					state = 2;
				}
				else {
					read_dynamixel = 0;
					if (readReference()){
						log(Debug) << "DynamixelController: New reference recieved, sending goals" << endlog();
						state++;
					} else {
						state = 2;
						log(Debug) << "DynamixelController: Read position of dynamixel, id " << read_dynamixel+1 << endlog();
					}
				}
				break;
			case 4:
				log(Debug) << "DynamixelController: setting speed and goal position" << endlog();
				create_pos_goal_packet();
				dxl_tx_rx_packet();
				state = 2;
				break;
			default:
				state = 1;
		}
	} else if (commStatus == COMM_RXCORRUPT) {
		log(Debug) << "DynamixelController: status corrupt. Resending the instruction."<< endlog();
		state = pstate;
		commStatus = COMM_RXSUCCESS;
	} else if (commStatus == COMM_RXWAITING) {
		if (trial>TRIAL_MAX) {
			log(Debug) << "DynamixelController: waited too long for status. Resending the instruction."<< endlog();
			state = pstate;
			commStatus = COMM_RXSUCCESS;
		} else {
			dxl_rx_packet();
			trial++;
		}
	}
}

void DynamixelController::create_pos_goal_packet(void)
{
	int L = 4;	// Number of addresses to change
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, AX_GOAL_POSITION_L);					// Staring address
	dxl_set_txpacket_parameter(1, L);									// Number of addresses to change
	
	uint start_param = 2;
	for ( uint i=0; i<n_dynamixels; i++ ){
		dxl_set_txpacket_parameter(start_param, dynamixel_ids[i]);						// id
		dxl_set_txpacket_parameter(start_param+1, dxl_get_lowbyte(dynamixel_goal[i]));		// value for address 1
		dxl_set_txpacket_parameter(start_param+2, dxl_get_highbyte(dynamixel_goal[i]));		// value for address 2
		dxl_set_txpacket_parameter(start_param+3, dxl_get_lowbyte(dynamixel_speed[i]));		// value for address 3
		dxl_set_txpacket_parameter(start_param+4, dxl_get_highbyte(dynamixel_speed[i]));	// value for address 4
		start_param += L+1;
	}
	dxl_set_txpacket_length((L+1)*n_dynamixels+4);		// length = (L+1)*N+4, L:=Number of addresses, N:=Number of dynamixels
}

void DynamixelController::dxl_tx_rx_packet(void) 
{
	dxl_tx_packet();
	if (commStatus == COMM_TXSUCCESS) {
		dxl_rx_packet();
	}
}

void DynamixelController::dxl_tx_packet(void)
{
	commStatus = COMM_TXSUCCESS;

	unsigned char TxNumByte;
	unsigned char checksum = 0;

	gbInstructionPacket[0] = 0xff;
	gbInstructionPacket[1] = 0xff;
	for(int i=0; i<(gbInstructionPacket[LENGTH]+1); i++) checksum += gbInstructionPacket[i+2];
	gbInstructionPacket[gbInstructionPacket[LENGTH]+3] = ~checksum;
	
	TxNumByte = gbInstructionPacket[LENGTH] + 4;
	
	log(Debug) << "Sent instruction: ";
	for(int i=0; i<TxNumByte; i++) {
		log(Debug) << (unsigned int) gbInstructionPacket[i] << " ";
	}
	log(Debug) << endlog();
	
	instruction.channels.resize(2);  // 2 channels on beckhoff
	instruction.channels[0].datapacket.clear();
	instruction.channels[0].datapacket.resize(TxNumByte);
	instruction.channels[0].datasize = TxNumByte;
	
	for (int i=0; i<TxNumByte; i++) {
		instruction.channels[0].datapacket[i]=gbInstructionPacket[i];
	}
	instructionPort.write(instruction);
}

void DynamixelController::dxl_rx_packet(void) 
{
	if (gbInstructionPacket[ID] == BROADCAST_ID) {
		log(Debug) << "DynamixelController: broadcast id, no status packet"<< endlog();
		commStatus = COMM_RXSUCCESS;
		return;
	}

	if (!(gbInstructionPacket[INSTRUCTION] == INST_READ)) {
		log(Debug) << "DynamixelController: not read instruction, no status packet"<< endlog();
		commStatus = COMM_RXSUCCESS;
		return;
	}
	
	if (!(statusPort.read(status) == NewData)) {		
		log(Debug) << "DynamixelController: waiting for data"<< endlog();
		commStatus = COMM_RXWAITING;
		return;
	} else {
		log(Debug) << "Received status: ";
		gbStatusSize = status.channels[0].datasize;
		for (int i=0; i<gbStatusSize; i++) {		
			log(Debug) << (unsigned int) status.channels[0].datapacket[i] << " ";
			gbStatusPacket[i] = status.channels[0].datapacket[i];
			
		}
		log(Debug) << endlog();
		
		unsigned char tmpLength = 6;
		if( gbInstructionPacket[INSTRUCTION] == INST_READ )
			tmpLength += gbInstructionPacket[PARAMETER+1];
		
		if ((gbStatusSize >= 6) && (gbStatusPacket[0] == 0xff) && (gbStatusPacket[1] == 0xff) && dxl_check_rxpacket_checksum()) {
			if ((gbStatusPacket[ID] != gbInstructionPacket[ID]) || (gbStatusSize != tmpLength)){
				log(Debug) << "DynamixelController: not my status packet or length not valid. waiting another"<< endlog();
				commStatus = COMM_RXWAITING;
			} else {
				log(Debug) << "DynamixelController: status packet ok"<< endlog();
				printErrorCode();
				commStatus = COMM_RXSUCCESS;
			}
		} else {
			log(Warning) << "Invalid status packet."<< endlog();
			commStatus = COMM_RXCORRUPT;
		}
	}
}

void DynamixelController::dxl_set_txpacket_id( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
}

void DynamixelController::dxl_set_txpacket_instruction( int instruction )
{
	gbInstructionPacket[INSTRUCTION] = (unsigned char)instruction;
}

void DynamixelController::dxl_set_txpacket_parameter( int index, int value )
{
	gbInstructionPacket[PARAMETER+index] = (unsigned char)value;
}

void DynamixelController::dxl_set_txpacket_length( int length )
{
	gbInstructionPacket[LENGTH] = (unsigned char)length;
}

int DynamixelController::dxl_get_rxpacket_id(void) 
{
	return (int)gbStatusPacket[ID];
}

int DynamixelController::dxl_rxpacket_isError(void)
{
	if(gbStatusPacket[ERRBIT]){
		errortosupervisorPort.write(true);
		return 1;
	}
	
	return 0;
}

int DynamixelController::dxl_get_rxpacket_error( int errbit )
{
	if( gbStatusPacket[ERRBIT] & (unsigned char)errbit )
		return 1;

	return 0;
}

int DynamixelController::dxl_get_rxpacket_length(void)
{
	return (int)gbStatusPacket[LENGTH];
}

bool DynamixelController::dxl_check_rxpacket_checksum(void)
{
	unsigned char checksum = 0;
	
	for(int i=0; i<(gbStatusPacket[LENGTH]+1); i++) 
		checksum += gbStatusPacket[i+2];
	checksum = ~checksum;
	return (gbStatusPacket[gbStatusPacket[LENGTH]+3] == checksum);
}

int DynamixelController::dxl_get_rxpacket_parameter( int index )
{
	return (int)gbStatusPacket[PARAMETER+index];
}

int DynamixelController::dxl_makeword( int lowbyte, int highbyte )
{
	unsigned short word;

	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int DynamixelController::dxl_get_lowbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff;
	return (int)temp;
}

int DynamixelController::dxl_get_highbyte( int word )
{
	unsigned short temp;

	temp = word & 0xff00;
	temp = temp >> 8;
	return (int)temp;
}

void DynamixelController::dxl_ping( int id )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_PING;
	gbInstructionPacket[LENGTH] = 2;
	
	dxl_tx_rx_packet();
}

void DynamixelController::dxl_read_byte( int id, int address )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 1;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void DynamixelController::dxl_write_byte( int id, int address, int value )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)value;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void DynamixelController::dxl_read_word( int id, int address )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_READ;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = 2;
	gbInstructionPacket[LENGTH] = 4;
	
	dxl_tx_rx_packet();
}

void DynamixelController::dxl_write_word( int id, int address, int value )
{
	gbInstructionPacket[ID] = (unsigned char)id;
	gbInstructionPacket[INSTRUCTION] = INST_WRITE;
	gbInstructionPacket[PARAMETER] = (unsigned char)address;
	gbInstructionPacket[PARAMETER+1] = (unsigned char)dxl_get_lowbyte(value);
	gbInstructionPacket[PARAMETER+2] = (unsigned char)dxl_get_highbyte(value);
	gbInstructionPacket[LENGTH] = 5;
	
	dxl_tx_rx_packet();
}

void DynamixelController::printErrorCode(void)
{
	int id = dxl_get_rxpacket_id();
	if (dxl_rxpacket_isError()) {
		bool unknown_id = true;
		for ( uint i=0; i<n_dynamixels; i++ ){
			if (id == dynamixel_ids[i]){
				log(Warning) << " Dynamixel " << dynamixel_ids[i] <<": ";
				unknown_id = false;
			}
		}
		if (unknown_id) {
			log(Warning) << "UNKNOWN Dynamixel id: ";
		}
	}
	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1) {
		log(Warning) << "Input voltage error!" << endlog();
	}

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1) { 
		log(Warning) << "Angle limit error!" << endlog();
	}

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1) {
		log(Warning) << "Overheat error!" << endlog();
	}

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1) {
		log(Warning) << "Out of range error!" << endlog();
	}
		
	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1) {
		log(Warning) << "Checksum error!" << endlog();
	}
		
	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1) {
		log(Warning) << "Overload error!" << endlog();
	}
			
	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1) {
		log(Warning) << "Instruction code error!" << endlog();
	}
}

ORO_CREATE_COMPONENT(DYNAMIXEL::DynamixelController)
