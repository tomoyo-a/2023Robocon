/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2020/12/28
  * @brief	 2021国赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *				None
  ******************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------*/

	#include "moveBase.h"
	#include "calculate.h"
	#include "pps.h"
  	#include "robot.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/
driverMsg_t oneWheelMsg={0},twoWheelMsg={0},thrWheelMsg={0},oneTurnMsg={0},twoTurnMsg={0},thrTurnMsg={0};
driverMsg_t oneWheelMsgCanRec={0},twoWheelMsgCanRec={0},thrWheelMsgCanRec={0},oneTurnMsgCanRec={0},twoTurnMsgCanRec={0},thrTurnMsgCanRec={0};
motorTargetMsg_t oneWheelTargetMsg={0},twoWheelTargetMsg={0},thrWheelTargetMsg={0},oneTurnTargetMsg={0},twoTurnTargetMsg={0},thrTurnTargetMsg={0};

//记录各个轮子朝向变量
static float oneAng = ONE_TURN_DIR, twoAng = TWO_TURN_DIR, threeAng = THR_TURN_DIR;

extern FILE *fpWrite;

/**
* @brief  将轮子速度转换为动力电机的脉冲
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
int WheelVel2MotorPulse(float vel,float turnMotorVelPulse)
{
	int pulse = 0;
	pulse = Vel2Pulse(vel) + turnMotorVelPulse;
	return pulse;
}
/**
* @brief  将动力电机的脉冲转化为轮子速度
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
float MotorPulse2WheelVel(float velMotorVelPulse,float turnMotorVelPulse)
{
	float vel = 0;
	vel = Pulse2Vel(velMotorVelPulse) - Pulse2Vel(turnMotorVelPulse);
	return vel;
}
/**
* @brief  Vel2Pulse将速度转换为脉冲
* @note
* @param  vel:速度（mm/s）
* @retval 脉冲速度
*/
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*MOTOR_PULSE_PER_ROUND*WHEEL_RATIO);
}
/**
* @brief  Pulse2Vel将速度转换为脉冲
* @note
* @param  pulse:脉冲速度
* @retval 速度（mm/s）
*/
float Pulse2Vel(int pulse)
{
	return ((float)pulse/MOTOR_PULSE_PER_ROUND)/WHEEL_RATIO*PI*WHEEL_DIAMETER;
}
/**
* @brief 将航向角度转换为脉冲
* @note
* @param  angle:°
* @retval 
*/
int TurnAngle2Pulse(float angle)
{
	int pulse = 0;	
	pulse = angle/360.0f*MOTOR_PULSE_PER_ROUND*TURN_RATIO + RESET_POS_ONE;
	return pulse;
}
/**
* @brief	得到实际航向电机位置函数
* @note		None
* @param	pulse 脉冲
* @retval	返回此时的角度，单位：度
*/
float Pulse2TurnAngle(int pulse)
{
	float angle = 0;
	angle = (pulse - RESET_POS_ONE)/TURN_RATIO/MOTOR_PULSE_PER_ROUND*360.0f;
	return angle;
}



/**
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
* @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
* @retval 
*/
void OutputVel2Wheel(float vel, float direction, float omega)
{
	wheel_t outputVel = {0.0f};
	
	//限幅
	if(vel>GetVelMax())
	{
		vel = GetVelMax();
	}
	else if(vel<0.0f)
	{
		vel = 0.0f;
	}

	
	if(omega > 240.0f)
	{
		omega = 240.0f;
	}
	else if(omega < -240.0f)
	{
		omega = -240.0f;
	}

	gRobot.robotVel = vel;
	gRobot.velDir = direction;
	gRobot.omg = omega;

	gRobot.everOmg =  Get_AverOmg(gRobot.omg);
    gRobot.everVel = Get_AverVel(gRobot.robotVel);
    gRobot.everVelDirec = Get_AverVelDirec(gRobot.velDir);

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d ", (int)(vel) , (int)(direction) , (int)(omega));
#endif
	//计算每个轮子的和速度大小和方向
	outputVel.one = CalcWheelSpeed(vel , direction , omega , GetAngle(), 1);
	
	outputVel.two = CalcWheelSpeed(vel , direction , omega , GetAngle(), 2);
	
	outputVel.three = CalcWheelSpeed(vel , direction , omega , GetAngle(), 3);

	outputVel.four = CalcWheelSpeed(vel , direction , omega , GetAngle(), 4);

	// fprintf(fpWrite, "w1ori %d %d %d o %d ", (int)outputVel.one.direction, (int)outputVel.two.direction, (int)outputVel.three.direction,(int)omega);
	
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"WW %d %d %d %d ", (int)(outputVel.leftFront.direction) , (int)(outputVel.rightFront.direction) ,\
				(int)(outputVel.leftRear.direction) , (int)(outputVel.rightRear.direction));
#endif
	
	//将和速度输出
	WheelVelControl(outputVel);
}

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */

void WheelVelControl(wheel_t wheelVel)
{
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif
	
	//将定位系统坐标系下角度转换为机器人坐标系下角度 direction-=GetAngle()
	Transform2RobotCoodinate(&wheelVel);
// fprintf(fpWrite, "w1oriRo %d %d %d ", (int)wheelVel.one.direction, (int)wheelVel.two.direction, (int)wheelVel.three.direction);
	
#ifdef SEND_MOVEBASE_DEBUGINFO	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	

// 	//将机器人坐标系下角度转换为和电机一致 direction = 90.0f - direction
// 	Transform2WheelCoodinate(&wheelVel);

// #ifdef SEND_MOVEBASE_DEBUGINFO	
// 	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// 			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
// 				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
// #endif	

	//判断是否需要将轮速反向
	JudgeVelDirection(&wheelVel.one, oneAng);
	JudgeVelDirection(&wheelVel.two, twoAng);
	JudgeVelDirection(&wheelVel.three, threeAng);

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	
	//保证旋转为劣弧
	oneAng = TurnInferiorArc(wheelVel.one.direction , oneAng);
	twoAng = TurnInferiorArc(wheelVel.two.direction , twoAng);
	threeAng = TurnInferiorArc(wheelVel.three.direction , threeAng);
// fprintf(fpWrite, "w1RoFI %d %d %d ", (int)oneAng, (int)twoAng, (int)threeAng);

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d \r\n", (int)(leftFrontAng) , (int)(rightFrontAng) ,\
				(int)(leftRearAng) , (int)(rightRearAng));
#endif
	
	
	SendCmd2Driver(wheelVel.one.vel , oneAng , wheelVel.two.vel , twoAng,
				 wheelVel.three.vel , threeAng);
	
}

/**
* @brief  SendCmd2Driver向电机发送速度和位置命令
* @note
* @param  lfVel:左前轮速度
		  lfDir：左前轮方向
		  rfVel:右前轮速度
		  rfDir：右前轮方向
		  lrVel:左后轮速度
		  lrDir：左后轮方向
		  rrVel:右后轮速度
		  rrDir：右后轮方向

* @retval 
*/
void SendCmd2Driver(float oneVel , float oneDir , float twoVel , float twoDir,
					float threeVel , float threeDir)
{
	//记录各个轮子实际给出的控制量
	gRobot.wheelState.oneTarget.vel = oneVel;
	gRobot.wheelState.oneTarget.direction = oneDir;
	
	gRobot.wheelState.twoTarget.vel = twoVel;
	gRobot.wheelState.twoTarget.direction = twoDir; 
	
	gRobot.wheelState.thrTarget.vel = threeVel;
	gRobot.wheelState.thrTarget.direction = threeDir;

	if(fabs(gRobot.wheelState.oneTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.oneTarget.vel = gRobot.wheelState.oneTarget.vel/fabs(gRobot.wheelState.oneTarget.vel)*LIMIT_VEL;
	}
	if(fabs(gRobot.wheelState.twoTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.twoTarget.vel = gRobot.wheelState.twoTarget.vel/fabs(gRobot.wheelState.twoTarget.vel)*LIMIT_VEL;
	}
	if(fabs(gRobot.wheelState.thrTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.thrTarget.vel = gRobot.wheelState.thrTarget.vel/fabs(gRobot.wheelState.thrTarget.vel)*LIMIT_VEL;
	}
	//can 发送电机指令 改为spi通信	
}

/**
* @brief  WheelLockTransform将轮子朝向机器人的中心
* @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
*/
void WheelLockTransform(float oneTargetDir, float twoTargetDir, float thrTargetDir)
{
	wheel_t outputVel = {0.0f};
	//计算每个轮子的和速度大小和方向
	outputVel.one = CalcWheelSpeed(0.0f , (GetAngle() + oneTargetDir) , 0.0f , GetAngle(), 1);
	
	outputVel.two = CalcWheelSpeed(0.0f , (GetAngle() + twoTargetDir) , 0.0f , GetAngle(), 2);
	
	outputVel.three = CalcWheelSpeed(0.0f , (GetAngle() + thrTargetDir) , 0.0f , GetAngle(), 3);

	//将和速度输出
	WheelVelControl(outputVel);
}

/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
* @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
*/
int WheelAngle2PositionTransform(float angle)
{
	return (int)(((angle / 360.0f)* TURN_RATIO)* MOTOR_PULSE_PER_ROUND );
}
/**
* @brief  WheelAngle2PositionInverseTransform将轮子脉冲位置转化为角度
* @note
* @param  position:轮子脉冲位置
* @retval 轮子朝向角度
*/
float WheelAngle2PositionInverseTransform(int position)
{
	return (float)(((float)position / MOTOR_PULSE_PER_ROUND)/ TURN_RATIO * 360.0f);
}
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
* @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
*/
void Transform2RobotCoodinate(wheel_t * wheelVel)
{
	//将定位系统坐标系下角度转换为机器人坐标系下角度
	wheelVel->one.direction-=GetAngle();
	wheelVel->two.direction-=GetAngle();
	wheelVel->three.direction-=GetAngle();

	//将角度限制在180度到-180度范围内
	AngleLimit(&wheelVel->one.direction);
	AngleLimit(&wheelVel->two.direction);
	AngleLimit(&wheelVel->three.direction);

}
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
* @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
*/
void Transform2WheelCoodinate(wheel_t * wheelVel)
{
	//将机器人坐标系下轮子朝向转换为轮子坐标系下角度
	wheelVel->one.direction = 90.0f - wheelVel->one.direction;
	wheelVel->two.direction = 90.0f - wheelVel->two.direction;
	wheelVel->three.direction = 90.0f - wheelVel->three.direction;
	
	//将角度限制在-180°到180°
	AngleLimit(&wheelVel->one.direction);
	AngleLimit(&wheelVel->two.direction);
	AngleLimit(&wheelVel->three.direction);

}
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
* @note
* @param  angle:要限制的值
* @retval 
*/
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}

	recursiveTimes--;
}
/**
* @brief  ReturnLimitAngle返回限制后的角度值
* @note
* @param  angle:要限制的值
* @retval 
*/
float ReturnLimitAngle(float angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(angle>180.0f)
		{
			angle = ReturnLimitAngle(angle - 360.0f);
		}
		else if(angle<-180.0f)
		{
			angle = ReturnLimitAngle(angle + 360.0f);
		}
	}
	
	recursiveTimes--;
	
	return angle;
}


/**
* @brief  JudgeVelDirection判断轮子是否需要反转
* @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子正方向角度
* @retval 
*/
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle)
{
	int n = 0;
	float angleErr = 0.0f;
	
	//将目标角度和当前实际角度转换到一个360度周期中
	n = (int)(actualAngle/180.0f) - (int)(actualAngle/360.0f);
	
	targetVel->direction = n * 360.0f + targetVel->direction;
	
	//计算目标角度和实际角度的误差
	angleErr = targetVel->direction - actualAngle;
	
	//将误差限制在-180度到180度
	AngleLimit(&angleErr);
	
	//如果角度误差大于90度则将速度反向并将目标角度加180度
	if(fabs(angleErr)>90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;
		
		//保证处理后的目标角度和当前实际角度在一个周期中
		if(targetVel->direction>(n * 360.0f + 180.0f))
		{
			targetVel->direction -=360.0f;
		}
		else if(targetVel->direction<(n * 360.0f - 180.0f))
		{
			targetVel->direction+=360.0f;
		}
	}
}

/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
* @note
* @param  targetAngle:目标角度
  @param  actualAngle：当前角度
* @retval 
*/
float TurnInferiorArc(float targetAngle , float actualAngle)
{
	if(targetAngle - actualAngle>180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if(targetAngle - actualAngle<-180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}	
}

/**
* @brief  CalcWheelSpeed
* @note	  计算轮子的和速度大小
* @param  vel:平移速度大小（mm/s）
* @param  direction:平移速度方向（-180°到180°）
* @param  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
* @param  nowAngle:当前姿态角
* @param  wheelNum:轮子序号
* @retval 
*/
wheelVel_t CalcWheelSpeed(float vel, float direction, float omega,float nowAngle, uint8_t wheelNum)
{
	wheelVel_t sumVel;
	float velX, velY = 0.0f;
	float velN = 0.0f;
	float angle = direction - nowAngle;
	if (angle > 180.f)
	{
		angle = angle - 360.f;
	}
	else if (angle < -180.f)
	{
		angle = angle + 360.f;
	}
	angle = angle/57.3f;
	//计算平移速度的X，Y分量
	velX = vel * cosf(angle);
	velY = vel * sinf(angle);
	//计算旋转的线速度
	velN = ANGLE2RAD(omega) * MOVEBASE_RADIUS;
	//计算合成各轮速
	switch (wheelNum)
	{
		case WHEEL_ONE:
		{
			sumVel.vel = velX + velY - (MOVEBASE_RADIUS + SAME_SIDE_WHELL_DIS/2.f)/MOVEBASE_RADIUS * velN;
			break;
		}
		case WHEEL_TWO:
		{
			sumVel.vel = -1.f * velX + velY - (MOVEBASE_RADIUS - SAME_SIDE_WHELL_DIS/2.f)/MOVEBASE_RADIUS * velN;
			break;
		}
		case WHEEL_THR:
		{
			sumVel.vel = velX + velY + (MOVEBASE_RADIUS - SAME_SIDE_WHELL_DIS/2.f)/MOVEBASE_RADIUS * velN;
			break;
		}
		case WHEEL_FOUR:
		{
			sumVel.vel = -1.f * velX + velY + (MOVEBASE_RADIUS + SAME_SIDE_WHELL_DIS/2.f)/MOVEBASE_RADIUS * velN;
			break;
		}
		default:
			break;
	}
	sumVel.direction = 45.f;
	return sumVel;
}

uint8_t TurnReset(void)
{
	static uint16_t resetCnt=0;
	static unsigned char turnFinishFlag = 0;
	if(oneTurnMsg.vel < 5000 && twoTurnMsg.vel < 5000)
	{
		resetCnt ++;
	}
	if( resetCnt > 50 && !turnFinishFlag)//停下一定时间则判断识别到磁铁，可以输出正确的位置了
	{
		turnFinishFlag = 1;
		resetCnt = 0;
	}
	else
	{
		oneWheelTargetMsg.velPulse = WheelVel2MotorPulse(0.0f,oneTurnMsg.vel);
		oneTurnTargetMsg.velPulse = 5*4096;
		twoWheelTargetMsg.velPulse = WheelVel2MotorPulse(0.0f,twoTurnMsg.vel);
		twoTurnTargetMsg.velPulse = 5*4096;
		thrWheelTargetMsg.velPulse = WheelVel2MotorPulse(0.0f,thrTurnMsg.vel);
		thrTurnTargetMsg.velPulse = 5*4096;
		VelCtrl(CAN0,PTP_MODE,ONE_WHEEL_ID,oneWheelTargetMsg.velPulse);
		VelCtrl(CAN0,PTP_MODE,ONE_TURN_ID,oneTurnTargetMsg.velPulse);
		VelCtrl(CAN0,PTP_MODE,TWO_WHEEL_ID,twoWheelTargetMsg.velPulse);
		VelCtrl(CAN0,PTP_MODE,TWO_TURN_ID,twoTurnTargetMsg.velPulse);
		VelCtrl(CAN0,PTP_MODE,THR_WHEEL_ID,thrWheelTargetMsg.velPulse);
		VelCtrl(CAN0,PTP_MODE,THREE_TURN_ID,thrTurnTargetMsg.velPulse);
	}
	if(turnFinishFlag)
	{
		resetCnt = 0;
		return 1;
	}
	return 0;
}

/**
* @brief	运动时读取电机信息函数
* @note  	None
* @param	None
* @retval	None
*/
void ReadMotorMsg(void)
{
	//读电机的速度
	ReadVel(CAN0, PTP_MODE, ONE_WHEEL_ID);
	ReadVel(CAN0, PTP_MODE, ONE_TURN_ID);
	ReadVel(CAN0, PTP_MODE, TWO_WHEEL_ID);
	ReadVel(CAN0, PTP_MODE, TWO_TURN_ID);
	ReadVel(CAN0, PTP_MODE, THR_WHEEL_ID);
	ReadVel(CAN0, PTP_MODE, THREE_TURN_ID);
	//读电机位置
	ReadPos(CAN0, PTP_MODE, ONE_TURN_ID);	
	ReadPos(CAN0, PTP_MODE, TWO_TURN_ID);	
	ReadPos(CAN0, PTP_MODE, THREE_TURN_ID);	
}


/**
 * @brief	位置初始化函数
 * @note	此函数为复位完成后转到初始位置
 * @param	None
 * @retval	到位置为1
		    未到为0
*/
uint8_t PosInit(void)
{
	float oneTurnPosAngle = 0;
	float twoTurnPosAngle = 0;
	float thrTurnPosAngle = 0;

	oneTurnPosAngle = Pulse2TurnAngle(oneTurnMsg.pos);
	twoTurnPosAngle = Pulse2TurnAngle(twoTurnMsg.pos);
	thrTurnPosAngle  = Pulse2TurnAngle(thrTurnMsg.pos);
	//判断是否转到位置，角度小于1.0°，并且速度小于50脉冲
	if((fabs(oneTurnPosAngle - ONE_POS_ANGLE_INIT) < 1.0f && abs(oneTurnMsg.vel) < 50) ||/*!!!&&*/\
	   (fabs(twoTurnPosAngle - TWO_POS_ANGLE_INIT) < 1.0f && abs(twoTurnMsg.vel) < 50) ||
	   (fabs(thrTurnPosAngle - THR_POS_ANGLE_INIT) < 1.0f && abs(thrTurnMsg.vel) < 50))
	{
		if(gRobot.robotMode == RESET_MODE)
		{
			SetCtrlMode(CAN0,PTP_MODE,TWO_TURN_ID,SPD_CURR_CTRL_MODE);
			SetCtrlMode(CAN0,PTP_MODE,ONE_TURN_ID,SPD_CURR_CTRL_MODE);
			SetCtrlMode(CAN0,PTP_MODE,THREE_TURN_ID,SPD_CURR_CTRL_MODE);
		}
		return 1;
	}
	oneTurnTargetMsg.posPulse =TurnAngle2Pulse(ONE_POS_ANGLE_INIT);
	twoTurnTargetMsg.posPulse =TurnAngle2Pulse(TWO_POS_ANGLE_INIT);
	thrTurnTargetMsg.posPulse =TurnAngle2Pulse(THR_POS_ANGLE_INIT);
	oneWheelTargetMsg.wheelRealVelTarget = 0.0f;
	twoWheelTargetMsg.wheelRealVelTarget = 0.0f;
	thrWheelTargetMsg.wheelRealVelTarget = 0.0f;
	oneWheelTargetMsg.velPulse = WheelVel2MotorPulse(oneWheelTargetMsg.wheelRealVelTarget, oneTurnMsg.vel);
	twoWheelTargetMsg.velPulse = WheelVel2MotorPulse(twoWheelTargetMsg.wheelRealVelTarget, twoTurnMsg.vel);
	thrWheelTargetMsg.velPulse = WheelVel2MotorPulse(thrWheelTargetMsg.wheelRealVelTarget, thrTurnMsg.vel);
	//初始化位置
	PosCtrl(CAN0, PTP_MODE, ONE_TURN_ID, ABSOLUTE_MODE, oneTurnTargetMsg.posPulse);	
	PosCtrl(CAN0, PTP_MODE, TWO_TURN_ID, ABSOLUTE_MODE, twoTurnTargetMsg.posPulse);	
	PosCtrl(CAN0, PTP_MODE, THREE_TURN_ID, ABSOLUTE_MODE, thrTurnTargetMsg.posPulse);	
	VelCtrl(CAN0, PTP_MODE, ONE_WHEEL_ID, oneWheelTargetMsg.velPulse);
	VelCtrl(CAN0, PTP_MODE, TWO_WHEEL_ID, twoWheelTargetMsg.velPulse);
	VelCtrl(CAN0, PTP_MODE, THR_WHEEL_ID, thrWheelTargetMsg.velPulse);
	return 0;
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
