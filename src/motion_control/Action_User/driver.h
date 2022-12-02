#ifndef __DRIVER_H
#define __DRIVER_H

#include "can.h"
#include "math.h"
#include "stdlib.h"

/*驱动器发送ID基地址*/
#define DRIVER_CLIENT_BASE_ID	0x280

/*驱动器接收ID基地址*/
#define DRIVER_SERVER_BASE_ID	0x300

/*控制标识符*/
#define IDENTIFIER_DRIVER_STATE				0x01
#define IDENTIFIER_CURR_KP_Q				0x02
#define IDENTIFIER_CURR_KI_Q				0x03
#define IDENTIFIER_SPD_KP					0x04
#define IDENTIFIER_SPD_KI					0x05
#define IDENTIFIER_POS_KP					0x06
#define IDENTIFIER_POS_KD					0x07
#define IDENTIFIER_TORQUE_CTRL				0x08
#define IDENTIFIER_VEL_CTRL					0x09
#define IDENTIFIER_POS_CTRL_ABS				0x0A
#define IDENTIFIER_POS_CTRL_REL				0x0B
#define IDENTIFIER_SET_CTRL_MODE			0x0C
#define IDENTIFIER_SET_ACC					0x0D
#define IDENTIFIER_SET_DEC					0x0E
#define IDENTIFIER_SET_TORQUE_LIMIT			0x0F
#define IDENTIFIER_SET_VEL_LIMIT			0x10
#define IDENTIFIER_SET_POS_LIMIT_UP			0x11
#define IDENTIFIER_SET_POS_LIMIT_LOW		0x12

/*读取标识符*/	
#define IDENTIFIER_READ_TORQUE				0x20
#define IDENTIFIER_READ_VEL					0x21
#define IDENTIFIER_READ_POS					0x22
#define IDENTIFIER_READ_ENCODER_POS			0x23
#define IDENTIFIER_READ_VOL_D				0x24
#define IDENTIFIER_READ_CURR_D				0x25
#define IDENTIFIER_READ_VOL_Q				0x26
#define IDENTIFIER_READ_CURR_Q				0x27
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT		0x28
#define IDENTIFIER_READ_POS_LOOP_OUTPUT		0x29

/*错误标识符*/
#define IDENTIFIER_ENCODER_ERROR		0xEE
#define IDENTIFIER_HARD_FAULT			0xFF

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

/*驱动器指令模式*/
typedef enum{
				PTP_MODE = 0x00, 
				BROADCAST_MODE = 0x40
			} CommandMode;

/*驱动器控制模式*/
typedef enum{
			SPD_CURR_CTRL_MODE = 1, 
			POS_SPD_CURR_CTRL_MODE = 2, 
			POS_CURR_CTRL_MODE = 3,
			TORQUE_CTRL_MODE = 4
			} ControlMode;

/*位置环模式*/
typedef enum{
			ABSOLUTE_MODE = IDENTIFIER_POS_CTRL_ABS, 
			RELATIVE_MODE = IDENTIFIER_POS_CTRL_REL
			} PosLoopMode;

/*主控接收驱动器消息内容结构体*/
typedef struct
{
	int vel;
	int pos;
	int torque;
	int volQ;
	int currQ;
	int volD;
	int currD;
	int encoder;
	int velLoopOut;
	int posLoopOut;
	uint8_t encoderErr;
	uint8_t hardFault;
	
}driverMsg_t;

/*******************************控制驱动器命令************************************/

/**
* @brief  驱动器状态(使能或失能)
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  state：状态, 范围: 
							ENABLE:使能
							DISABLE:失能
* @author ACTION
* @note 
*/
void DriverState(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state);

/**
* @brief  电机转矩控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  torque: 转矩，单位：毫牛米, 范围：-100 * 1000 ~ 100 * 1000
* @author ACTION
*/
void TorqueCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t torque);

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  vel: 速度，单位：脉冲每秒, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void VelCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t vel);

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @author ACTION
*/
void PosCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, PosLoopMode posMode, int32_t pos);

/**
* @brief  配置驱动器控制模式
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  ctrlMode：控制模式，范围：
			SPD_CURR_CTRL_MODE：速度-电流双闭环模式
			POS_SPD_CURR_CTRL_MODE：位置-速度-电流三闭环模式
			POS_CURR_CTRL_MODE：位置-电流双闭环模式
* @author ACTION
* @note
*/
void SetCtrlMode(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, ControlMode ctrlMode);

/**
* @brief  配置电机加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  acc：加速度，单位：脉冲每二次方秒, 范围：0 ~ 1024 * 4096
* @param  dec：减速度，单位：脉冲每二次方秒, 范围：0 ~ 1024 * 4096
* @author ACTION
* @note
*/
void SetAccDec(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec);

/**
* @brief  配置电机最大转矩
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：最大转矩, 单位: 毫牛米, 范围：0 ~ 100 * 1000
* @author ACTION
* @note
*/
void SetTorqueLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);
	
/**
* @brief  配置速度环最大期望速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  limit：速度限制，单位：脉冲每秒, 范围：0 ~ 1024 * 4096
* @author ACTION
* @note
*/
void SetVelLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit);

/**
* @brief  配置电机位置限制
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  upperLimit：正方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ 1024 * 4096
* @param  lowerLimit：反方向位置限制，单位：脉冲, 范围：-1024 * 4096 ~ upperLimit
* @author ACTION
* @note
*/
void SetPosLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t upperLimit, int32_t lowerLimit);

/**
* @brief  配置电流环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_current：电流环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_current);

/**
* @brief  配置电流环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_current：电流环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetCurrentKI(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_current);

/**
* @brief  配置速度环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_speed：速度环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed);

/**
* @brief  配置速度环ki
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  i_speed：速度环ki值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetSpeedKI(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed);

/**
* @brief  配置位置环kp
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  p_pos：位置环kp值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos);

/**
* @brief  配置位置环kd
* @param  CANx: 所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @param  d_pos：位置环kd值, 单位: 必须乘以1000发过去！！！
* @author ACTION
* @note
*/
void SetPosKD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos);

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机电磁转矩
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫牛米
*/
void ReadTorque(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVel(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机速度环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲每秒
*/
void ReadVelLoopOut(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPos(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机位置环输出
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadPosLoopOut(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取编码器脉冲
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：脉冲
*/
void ReadEncoder(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);
	
/**
* @brief  读取电机交轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolQ(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机转矩电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrQ(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);


/**
* @brief  读取电机直轴电压
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫伏
*/
void ReadVolD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  读取电机直轴电流
* @param  CANx：所使用的CAN通道编号
* @param  CMDmode: 指令模式，范围：
				PTP_MODE: 点对点模式
				BROADCAST_MODE: 广播模式
* @param  DriverNum：驱动器ID号(广播模式下为组号), 范围：1~64
* @author ACTION
* @note   单位：毫安
*/
void ReadCurrD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum);

/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便用户使用 
*/
void GetDriverMsg(driverMsg_t *driverMsg, uint8_t *data8);

/*将三个高字节的数据移到低三字节并提取符号位*/
int TransformValue(int data32);

/**
* @brief  读取航向位置
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：，
* @author ACTION
 * @note：
*/
void ReadActualPos(uint8_t canPort, uint8_t ID);
	
#endif


