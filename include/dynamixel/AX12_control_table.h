/// Created according to Robotis User's manual 2006-06-14 page 12
/// Can be used for both AX-12 as AX-64

// AX-12 manual: http://www.electronickits.com/robot/BioloidAX-12(english).pdf
// AX-64 manual: http://creativemachines.cornell.edu/sites/default/files/RX-64_Manual.pdf

// Control table address
#define AX_MODEL_NUMBER_L       0
#define AX_MODEL_NUMBER_H       1
#define AX_FIRMWARE             2
#define AX_ID                   3
#define AX_BAUD_RATE            4
#define AX_RETURN_DELAY_TIME    5
#define AX_CW_ANGLE_LIMIT_L     6
#define AX_CW_ANGLE_LIMIT_H     7
#define AX_CCW_ANGLE_LIMIT_L    8
#define AX_CCW_ANGLE_LIMIT_H    9
#define AX_TEMP_LIMIT_H         11
#define AX_VOLT_LIMIT_L         12
#define AX_VOLT_LIMIT_H         13
#define AX_MAX_TORQUE_L         14
#define AX_MAX_TORQUE_H         15
#define AX_STATUS_RETURN        16
#define AX_ALARM_LED            17
#define AX_ALARM_SHUTDOWN       18
#define AX_DOWN_CALIBRATION_L   20
#define AX_DOWN_CALIBRATION_H   21
#define AX_UP_CALIBRATION_L     22
#define AX_UP_CALIBRATION_H     23
#define AX_TORQUE_ENABLE        24
#define AX_LED                  25
#define AX_CW_COMP_MARGIN       26
#define AX_CCW_COM_MARGIN       27
#define AX_CW_COM_SLOPE         28
#define AX_CCW_COM_SLOPE        29
#define AX_GOAL_POSITION_L		30
#define AX_GOAL_POSITION_H		31
#define AX_MOVING_SPEED_L       32
#define AX_MOVING_SPEED_H       33
#define AX_TORQUE_LIMIT_L       34
#define AX_TORQUE_LIMIT_H       35
#define AX_PRESENT_POSITION_L	36
#define AX_PRESENT_POSITION_H	37
#define AX_PRESENT_SPEED_L      38
#define AX_PRESENT_SPEED_H      39
#define AX_PRESENT_LOAD_L       40
#define AX_PRESENT_LOAD_H       41
#define AX_PRESENT_VOLT         42
#define AX_PRESENT_TEMP         43
#define AX_REGISTERED_INSTR     44
#define AX_MOVING				46
#define AX_LOCK                 47
#define AX_PUNCH_L              48
#define AX_PUNCH_H              49

#define ID					(2)
#define LENGTH				(3)
#define INSTRUCTION			(4)
#define ERRBIT				(4)
#define PARAMETER			(5)
#define DEFAULT_BAUDNUMBER	(1)
#define MAXNUM_TXPARAM		(150)
#define MAXNUM_RXPARAM		(60)
#define BROADCAST_ID		(254)
#define INST_PING			(1)
#define INST_READ			(2)
#define INST_WRITE			(3)
#define INST_REG_WRITE		(4)
#define INST_ACTION			(5)
#define INST_RESET			(6)
#define INST_SYNC_WRITE		(131)
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL		(2)
#define COMM_RXFAIL		(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)
#define COMM_TXWAITING		(8)
#define COMM_TXTIMEOUT		(9)
