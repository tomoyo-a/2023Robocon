/*
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

/* Includes ------------------------------------------------------------------*/
	#include <stdint.h>
	#include <math.h>
 	#include <stdlib.h>
   	#include "driver.h"

//轮子序数
#define WHEEL_ONE    (1)
#define WHEEL_TWO    (2)
#define WHEEL_THR    (3)
#define WHEEL_FOUR   (4)

/* Exported types ------------------------------------------------------------*/
//机器人速度结构体
typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;

typedef struct
{
	//速度大小
	int velPulse;
  int posPulse;
  float wheelRealVelTarget;
}motorTargetMsg_t;

//轮子速度和方向结构体
typedef struct
{
	//轮子速度大小
	float vel;
	//轮子速度方向
	float direction;
}wheelVel_t;

//轮子结构体
typedef struct
{
	//左前轮
	wheelVel_t one;
	//右前轮
	wheelVel_t two;
	//左后轮
	wheelVel_t three;
  //左后轮
	wheelVel_t four;
}wheel_t;

//轮子状态结构体
typedef struct
{
	//1号轮目标速度与方向
	wheelVel_t oneTarget;
	//2号轮目标速度与方向
	wheelVel_t twoTarget;
	//3号轮目标速度与方向
	wheelVel_t thrTarget;
	
}wheelState_t;
 
/* Exported constants --------------------------------------------------------*/

/** @defgroup 
  * @{
  */

//#define SEND_MOVEBASE_DEBUGINFO

#ifndef PI
#define PI (3.1415926535f)
#endif

#define LIMIT_VEL (10000.0f)
#define ONE_TURN_DIR (-60.f)//初始轮子方向，三个轮子一致
#define TWO_TURN_DIR (0.f)
#define THR_TURN_DIR (60.f)
#define ORI_TURN_DIR (180.0f)
//轮子id号
#define ONE_WHEEL_ID (1)
#define ONE_TURN_ID (2)

#define TWO_WHEEL_ID (3)
#define TWO_TURN_ID (4)

#define THR_WHEEL_ID (5)
#define THREE_TURN_ID (6)

//动力齿轮减速比
#define WHEEL_RATIO				(4.0f/3.0f)
//舵向齿轮减速比
#define TURN_RATIO				(4.f/1.0f)
//电机转一圈的脉冲
#define MOTOR_PULSE_PER_ROUND			(4096.0f)
//电机加速度
#define ACC						(360.0f*300.0f)

//电机最大转速 脉冲为单位
#define MAX_MOTOR_SPEED (MOTOR_PULSE_PER_ROUND*120)

//定位系统到中心距离
#define DIS_OPS2CENTER (271.9f)

//底盘旋转半径
#define MOVEBASE_RADIUS (250.00f)
//同侧两轮之间距离
#define SAME_SIDE_WHELL_DIS (73.37f)
//轮子直径
#define WHEEL_DIAMETER			(101.6f)
//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)
//摄像头的z轴距离
#define L515_HEIGHT (199.f)


//
//轮与中心连线切线方向  nishizhen wei zheng
#define ONE_VERTICAL_ANG (-150.0f)
//轮与中心连线切线方向
#define TWO_VERTICAL_ANG (90.0f)
//轮与中心连线切线方向
#define THREE_VERTICAL_ANG (-30.0f)

//航向编码器复位位置，每换一次轮子电机要重测一次
//暂时未测
#define RESET_POS_ONE	(0.0f)
#define RESET_POS_TWO	(0.0f)
#define RESET_POS_THR	(0.0f)
#if OPERATING_MODE == NORMAL_MODE
//初始化位置(角度/-90~90/)
#define ONE_POS_ANGLE_INIT (-60.f)
#define TWO_POS_ANGLE_INIT (0.f)
#define THR_POS_ANGLE_INIT (60.f)
#endif
extern motorTargetMsg_t oneWheelTargetMsg,twoWheelTargetMsg,thrWheelTargetMsg,oneTurnTargetMsg,twoTurnTargetMsg,thrTurnTargetMsg;
extern driverMsg_t oneWheelMsg,twoWheelMsg,thrWheelMsg,oneTurnMsg,twoTurnMsg,thrTurnMsg;
extern driverMsg_t oneWheelMsgCanRec,twoWheelMsgCanRec,thrWheelMsgCanRec,oneTurnMsgCanRec,twoTurnMsgCanRec,thrTurnMsgCanRec;

//
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
* @brief  将轮子速度转换为动力电机的脉冲
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
int WheelVel2MotorPulse(float vel,float turnMotorVelPulse);
/**
* @brief  将动力电机的脉冲转化为轮子速度
* @note
* @param  vel:速度（mm/s）
* @param  turnMotorVelPulse:舵向电机速度脉冲
* @retval 脉冲速度
*/
float MotorPulse2WheelVel(float velMotorVelPulse,float turnMotorVelPulse);
/**
* @brief  Vel2Pulse将速度转换为脉冲
* @note
* @param  vel:速度（mm/s）
* @retval 脉冲速度
*/
int Vel2Pulse(float vel);
/**
* @brief  Pulse2Vel将速度转换为脉冲
* @note
* @param  pulse:脉冲速度
* @retval 速度（mm/s）
*/
float Pulse2Vel(int pulse);
/**
* @brief 将航向角度转换为脉冲
* @note
* @param  angle:°
* @retval 
*/
int TurnAngle2Pulse(float angle);
/**
* @brief	得到实际航向电机位置函数
* @note		None
* @param	pulse 脉冲
* @retval	返回此时的角度，单位：度
*/
float Pulse2TurnAngle(int pulse);
/**
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
  * @retval 
  */
void OutputVel2Wheel(float vel, float direction, float omega);

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */
void WheelVelControl(wheel_t wheelVel);

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
void SendCmd2Driver(float frontVel , float frontDir , float lrVel , float lrDir,
					float rrVel , float rrDir);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle);
/**
* @brief  WheelLockTransform将轮子朝向机器人的中心
* @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
*/
void WheelLockTransform(float oneTargetDir, float twoTargetDir, float thrTargetDir);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
float WheelAngle2PositionInverseTransform(int position);
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
  */
void Transform2RobotCoodinate(wheel_t * wheelVel);
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
  */
void Transform2WheelCoodinate(wheel_t * wheelVel);
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle);
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle);
/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle);
/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle);
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
wheelVel_t CalcWheelSpeed(float vel, float direction, float omega,float nowAngle, uint8_t wheelNum);

uint8_t TurnReset(void);

/**
* @brief	运动时读取电机信息函数
  * @note	None
  * @param	None
  * @retval	None
  */
void ReadMotorMsg(void);

/**
* @brief	判断是否到复位位置函数
  * @note	None
  * @param	None
  * @retval	到位置为1
			未到为0
  */
uint8_t PosInit(void);

#endif /* ___H */

/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/
