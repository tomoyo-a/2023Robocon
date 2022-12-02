#include "driver.h"
#include "can.h"
#include <stdio.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

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
void DriverState(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, FunctionalState state)
{
	uint16_t identifier = IDENTIFIER_DRIVER_STATE;
	uint32_t data = state;
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
		
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;	  
	TxMessage.can_dlc = 2;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;

    switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void TorqueCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t torque)
{ 
	uint16_t identifier = IDENTIFIER_TORQUE_CTRL;
	uint32_t data = abs(torque);
	uint16_t timeout = 0;
	uint8_t mbox;
	struct can_frame TxMessage;
	
	/*限幅*/
	if(data >= 100 * 1000)
	{
		data = 100 * 1000;
	}
					 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(torque >= 0)
	{
		TxMessage.data[3] = (data>>16)&0xFF; 
	}
	else if(torque < 0)
	{
		TxMessage.data[3] = ((data>>16)&0xFF) | 0x80;
	}

    switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void VelCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t vel)
{ 
	uint16_t identifier = IDENTIFIER_VEL_CTRL;
	uint32_t data = abs(vel);
	uint16_t timeout = 0;
	uint8_t mbox;
	struct can_frame TxMessage;
	
	/*限幅*/
	if(data >= 1024 * 4096)
	{
		data = 1024 * 4096;
	}
					 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(vel >= 0)
	{
		TxMessage.data[3] = (data>>16)&0xFF; 
	}
	else if(vel < 0)
	{
		TxMessage.data[3] = ((data>>16)&0xFF) | 0x80;
	}

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void PosCtrl(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, PosLoopMode posMode, int32_t pos)
{
	uint16_t identifier = posMode;
	uint32_t data = abs(pos);
	uint16_t timeout = 0;
	uint8_t mbox;
	struct can_frame TxMessage;
	
	/*限幅*/
	if(data >= 1024 * 4096)
	{
		data = 1024 * 4096;
	}

	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	
	/*发送符号位*/
	if(pos >= 0)
	{
		TxMessage.data[3] = (data>>16)&0xFF; 
	}
	else if(pos < 0)
	{
		TxMessage.data[3] = ((data>>16)&0xFF) | 0x80;
	}

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetCtrlMode(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, ControlMode ctrlMode)
{
	uint16_t identifier = IDENTIFIER_SET_CTRL_MODE;
	uint32_t data = ctrlMode;
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
					 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
	
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetAccDec(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t acc, uint32_t dec)
{
	uint16_t identifier[2] = {IDENTIFIER_SET_ACC, IDENTIFIER_SET_DEC};
	uint32_t data[2] = {acc, dec};
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;

	/*限幅*/
	for(uint8_t i = 0; i < 2; i++)
	{
		if(data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}
	
	for(uint8_t i = 0; i < 2; i++)
	{
		TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
		TxMessage.can_dlc = 4;
		TxMessage.data[0] = (identifier[i]>>0)&0xFF;
		TxMessage.data[1] = (data[i]>>0)&0xFF;
		TxMessage.data[2] = (data[i]>>8)&0xFF;
		TxMessage.data[3] = (data[i]>>16)&0xFF;
		
		switch(canPort)
        {
            case CAN0:
            {
                int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
                if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                    printf("CAN0 SEND ERROR");

                break;
            }
            case CAN1:
            {
                int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
                if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                    printf("CAN1 SEND ERROR");

                break;
            }
            default:
                break;
        }
	}
}

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
void SetTorqueLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	uint16_t identifier = IDENTIFIER_SET_TORQUE_LIMIT;
	uint32_t data = limit;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	if(data > 100 * 1000)
	{
		data = 100 * 1000;
	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetVelLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t limit)
{
	uint16_t identifier = IDENTIFIER_SET_VEL_LIMIT;
	uint32_t data = limit;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	if(data > 1024 * 4096)
	{
		data = 1024 * 4096;
	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetPosLimit(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, int32_t upperLimit, int32_t lowerLimit)
{
	uint16_t identifier[2] = {IDENTIFIER_SET_POS_LIMIT_UP, IDENTIFIER_SET_POS_LIMIT_LOW};
	int32_t limit[2] = {upperLimit, lowerLimit};
	uint32_t data[2] = {0};
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*upperLimit必须大于或等于lowerLimit*/
	if(limit[1] > limit[0])
	{
		limit[1] = limit[0];
	}
	
	data[0] = abs(limit[0]);
	data[1] = abs(limit[1]);
	
	/*限幅*/
	for(uint8_t i = 0; i < 2; i++)
	{
		if(data[i] > 1024 * 4096)
		{
			data[i] = 1024 * 4096;
		}
	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;	
	TxMessage.can_dlc = 4;
	
	for(uint8_t i = 0; i < 2; i++)
	{
		TxMessage.data[0] = (identifier[i]>>0)&0xFF;
		TxMessage.data[1] = (data[i]>>0)&0xFF;
		TxMessage.data[2] = (data[i]>>8)&0xFF;
			
		/*发送符号位*/
		if(limit[i] >= 0)
		{
			TxMessage.data[3] = (data[i]>>16)&0xFF;
		}
		else if(limit[i] < 0)
		{
			TxMessage.data[3] = ((data[i]>>16)&0xFF) | 0x80;
		}
			
		switch(canPort)
        {
            case CAN0:
            {
                int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
                if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                    printf("CAN0 SEND ERROR");

                break;
            }
            case CAN1:
            {
                int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
                if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                    printf("CAN1 SEND ERROR");

                break;
            }
            default:
                break;
        }
	}

}

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
void SetCurrentKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_current)
{
	uint16_t identifier = IDENTIFIER_CURR_KP_Q;
	uint32_t data = p_current;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetCurrentKI(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_current)
{
	uint16_t identifier = IDENTIFIER_CURR_KI_Q;
	uint32_t data = i_current;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetSpeedKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KP;
	uint32_t data = p_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetSpeedKI(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t i_speed)
{
	uint16_t identifier = IDENTIFIER_SPD_KI;
	uint32_t data = i_speed;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetPosKP(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t p_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KP;
	uint32_t data = p_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void SetPosKD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum, uint32_t d_pos)
{
	uint16_t identifier = IDENTIFIER_POS_KD;
	uint32_t data = d_pos;	
	uint8_t mbox;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
	
	/*限幅*/
	//fix me 添加正确的限幅
//	if(data > 100 * 1000)
//	{
//		data = 100 * 1000;
//	}
	
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 4;
	TxMessage.data[0] = (identifier>>0)&0xFF;
	TxMessage.data[1] = (data>>0)&0xFF;
	TxMessage.data[2] = (data>>8)&0xFF;
	TxMessage.data[3] = (data>>16)&0xFF;
		
	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void ReadTorque(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_TORQUE;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void ReadVel(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VEL;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;

	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}


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
void ReadVelLoopOut(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;

	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}


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
void ReadPos(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_POS;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void ReadPosLoopOut(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}


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
void ReadEncoder(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_ENCODER_POS;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void ReadVolQ(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VOL_Q;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}


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
void ReadCurrQ(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_Q;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

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
void ReadVolD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_VOL_D;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}


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
void ReadCurrD(uint8_t canPort, CommandMode CMDmode, uint8_t DriverNum)
{
	uint16_t identifier = 0x40 + IDENTIFIER_READ_CURR_D;
	uint8_t mbox = 0;
	uint16_t timeout = 0;
	struct can_frame TxMessage;
						 
	TxMessage.can_id = DRIVER_SERVER_BASE_ID + CMDmode + DriverNum;
	TxMessage.can_dlc = 1;
	TxMessage.data[0] = (identifier>>0)&0xFF;

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
}

/**
* @brief  获取电机消息的值
* @param  *driverMsg：对应电机消息结构体的地址
* @param  *data8: 接收驱动器消息数组的首地址
* @author ACTION
* @note   方便用户使用 
*/
void GetDriverMsg(driverMsg_t *driverMsg, uint8_t *data8)
{
	union receive
	{
		uint8_t data8[4];
		int data32;
		float dataf;
	}receiveMsg;
	
	for(int i=0; i<4; i++)
		receiveMsg.data8[i] = data8[i];
	
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VEL)
	{
		//获取脉冲速度
		driverMsg->vel = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_POS)
	{
		//获取脉冲位置
		driverMsg->pos = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_ENCODER_POS)
	{
		//获取编码器脉冲
		driverMsg->encoder = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_TORQUE)
	{
		//获取电磁转矩
		driverMsg->torque = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VOL_Q)
	{
		//获取交轴电压
		driverMsg->volQ = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_CURR_Q)
	{
		//获取转矩电流
		driverMsg->currQ = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_CURR_D)
	{
		//获取转矩电流
		driverMsg->currD = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_VOL_D)
	{
		//获取转矩电流
		driverMsg->volD = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_SPD_LOOP_OUTPUT)
	{
		//获取转矩电流
		driverMsg->velLoopOut = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_READ_POS_LOOP_OUTPUT)
	{
		//获取转矩电流
		driverMsg->posLoopOut = TransformValue(receiveMsg.data32);
	}
	if(receiveMsg.data8[0] == IDENTIFIER_ENCODER_ERROR)
	{
		driverMsg->encoderErr = 1;
	}
	if(receiveMsg.data8[0] == IDENTIFIER_HARD_FAULT)
	{
		driverMsg->hardFault = 1;
	}
}
/*将三个高字节的数据移到低三字节并提取符号位*/
int TransformValue(int data32)
{
	if((data32&0x80000000) == 0)
	{
		return ((data32>>8)&0x7FFFFF);
	}
	else
	{
		return(-((data32>>8)&0x7FFFFF)); 
	}
}



/**
* @brief  读取航向位置，大转盘位置
* @param  CANx：所使用的CAN通道编号
* @param  ID：驱动器ID号，范围：，
* @author ACTION
 * @note：
*/
void ReadActualPos(uint8_t canPort, uint8_t ID)
 {
	struct can_frame TxMessage;
						 
	TxMessage.can_id=DRIVER_SERVER_BASE_ID + ID;					// standard identifier=0
    TxMessage.can_dlc=8;


   	TxMessage.data[0] = 'A';
	TxMessage.data[1] = 'T';
	TxMessage.data[2] = 'g';
	TxMessage.data[3] = 'e';
	TxMessage.data[4] = 't';
	TxMessage.data[5] = 'p';
	TxMessage.data[6] = 'o';
	TxMessage.data[7] = 's';

	switch(canPort)
    {
        case CAN0:
        {
            int nbytes = write(socket_can0, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN0 SEND ERROR");

            break;
        }
        case CAN1:
        {
            int nbytes = write(socket_can1, &TxMessage, sizeof(TxMessage)); //发送数据
            if(nbytes != sizeof(TxMessage)) //如果 nbytes 不等于帧长度，就说明发送失败
                printf("CAN1 SEND ERROR");

            break;
        }
        default:
            break;
    }
 }
 
