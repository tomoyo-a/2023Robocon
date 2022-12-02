#include "posSystem.h"
#include "MotionCard.h"
#include <math.h>
#include <stdio.h>
#include "Bspline.h"
#include "robot.h"
#include "pathFollowing.h"
#include "pps.h"
/***********************************************************************************
* @name 		CaculatePath
* @brief  		计算走过的里程
* @param  	
* @retval 	
**********************************************************************************/
static float lengthWheelTraveled = 0.0f;
//0不记录
//1记录距离
//默认初始就开始记录距离
static _Bool CaculateLenFlag = 0;
float posXOld = 0.0f,posYOld = 0.0f;
extern FILE *fpWrite;
//通过定位系统计算机器人行走的路径长度
void CaculatePath(void)
{
	
	float err = -0.02f;
	float errDirection = 0.0f;
	float reducePath = 0.0f;
	PointU_t virtualPosition = {0.0f};
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetX() - posXOld)*(GetX() - posXOld) + (GetY() - posYOld)*(GetY() - posYOld));
		if(err> 0.2f)
		{
			errDirection = CHANGE_TO_ANGLE*atan2f((GetY() - posYOld) , (GetX() - posXOld));
			virtualPosition = SerchVirtualPoint2(GetPath());
			err = err * cosf(CHANGE_TO_RADIAN * (errDirection - virtualPosition.direction));
	
			if(err>=0.0f)
			{
				if(err<300.0f)
				{
					lengthWheelTraveled += err;
				}
				else
				{
					lengthWheelTraveled += err;
					fprintf(fpWrite, "ERR_TOO_BIG %d\r\n",(int)err);
					if(fabs(err) > 1000.f)
						gRobot.walkStatus = stop;
				}
			}
			if(lengthWheelTraveled<=0.0f)
			{
				lengthWheelTraveled = 0.0f;
			}
			posXOld = GetX();
			posYOld = GetY();
		}
	}
}

//运行路径长度记录
void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}

//停止路径长度记录
void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
}

//增加距离
void AddPath(float dis)
{
	lengthWheelTraveled += dis;
}

//减小距离
void ReducePath(float dis)
{
	lengthWheelTraveled -= (dis);
}

/***********************************************************************************
* @name 		GetPath
* @brief  	返回里程数
* @param  	无
* @retval 	lengthWheelTraveled
**********************************************************************************/
float GetPath(void)
{
	return lengthWheelTraveled;
}

/***********************************************************************************
* @name 		GetPosPresent
* @brief  	返回当前姿态
* @param  	无
* @retval 	
**********************************************************************************/
Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetX();
	pos.point.y = GetY();
	pos.direction   = GetAngle();
	pos.vel = sqrt(GetSpeedWithoutOmega().xVel*GetSpeedWithoutOmega().xVel + GetSpeedWithoutOmega().yVel*GetSpeedWithoutOmega().yVel);
	return pos;
}

void ClearPathLen(void)
{
	posXOld = GetX();
	posYOld = GetY();
	lengthWheelTraveled = 0.0f;
	UpdateLenBegin();
}
