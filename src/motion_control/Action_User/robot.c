#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include "robot.h"
#include "posSystem.h"
#include "path.h"
#include "pps.h"
#include "mcucomm.h"
#include "pathFollowing.h"
#include "ringbuffer.h"
#include "Move.h"
#include "moveBase.h"
#include "cv.h"
/*  ********************************************	计时	***************************************************** */
/*  ****************************记得所有在射箭区的轮子抱死******************/
static int timeCounter = 0;
static uint8_t countTimeFlag = 0;
static uint8_t stopCnt = 0;

extern robotVel_t targetVel;

extern FILE *fpWrite;

int GetTimeCounter(void)
{	
	return timeCounter; 
}

void CountTime(void)
{
	if(countTimeFlag)
	{
		timeCounter++;
	}
	timeCounter%=500000000;
}

void SetCountTimeFlag(void)
{
	countTimeFlag = 1;
}

/*  ********************************************Walk***************************************************** */
static int delayCount = 0;
static int firstPathFlag = 0;
static uint8_t waitMomentum = 0;	
void Walk(void)
{
	// gRobot.walkStatus = testPara;
	//mcu连续通信失败20次 停下
	printf("walk1\n");
	if( gRobot.mcuHeart > 20)
	{
		fprintf(fpWrite, "\n MCU ERR! ");
		printf(" MCU ERR! ");
		gRobot.walkStatus = stop;
	}
	//pps连续通信失败20次 停下
	if( gRobot.ppsHeart > 20)
	{
		fprintf(fpWrite, " pps ERR ");
		printf(" pps ERR ");
		gRobot.walkStatus = stop;
	}
	if( gRobot.ppsSameHeart > 20)
	{
		fprintf(fpWrite, " pps same ERR ");
		printf(" pps same ERR ");
		gRobot.walkStatus = stop;
	}
	printf("walk2\n");
	
	if(GetY() < 10.f)
	{
		gRobot.walkStatus = stop;
	}
	switch(gRobot.walkStatus)
	{
		//等待触发
		case waitForBlueTakeBall:
		{	
			stopCnt = 0;
			printf("walk3\n");
			//开始计时
			SetCountTimeFlag();
			// float initVelAngle = 0;//记得改回来
			// initVelAngle = GetAngle()+ORI_TURN_DIR;
			// AngleLimit(&initVelAngle);
			// OutputVel2Wheel(0.0f,initVelAngle,0.0f);
			float initVelAngle = 0;
			initVelAngle = GetAngle();
			AngleLimit(&initVelAngle);
			OutputVel2Wheel(0.0f,initVelAngle,0.0f);

			gRobot.pathPlanFlag = IS_PLANNING; //防止路径规划时间过长触发错误检测
			
			#ifdef SPI_COM
			Communicate2Mcu();
			#endif
			ClearRingBuffer();
			
			//规划第一段轨迹
			#ifdef SPI_COM
			BlueTakeBAllPath[0].point.x = GetX();
			BlueTakeBAllPath[0].point.y = GetY();
			BlueTakeBAllPath[0].direction = GetAngle();
			#endif
			InputPoints2RingBuffer(BlueTakeBAllPath, BLUE_TAKE_BALL_PATH_NUM);
			gRobot.pathPlanFlag = NO_PLANNING;
			gRobot.velControlStop = 1;
			gRobot.walkStatus = goForBlueTakeBall;
			ClearPathLen();//后清除记录的轨迹长度
			break;
		}

		//停止状态
		case stop:
		{
			
			static float angleAim = 0.f;
			float angleAimVel = 0.f;
			stopCnt++;
			if(stopCnt < 2)
			{
				angleAim = GetAngle();
			}
			else if(stopCnt > 3)
			{
				stopCnt = 3;
			}
			
			angleAimVel = AngleControl(GetPosPresent().direction,angleAim,4.f,25.f);
			OutputVel2Wheel(0.0f,180.0f,angleAimVel);
			if(GetY() > 10.f && GetY() < 260.f)
			{
				if(gRobot.cvData.filed == RED_COURT)
				{
					gRobot.walkStatus = waitForRedTakeBall;
				}
				else
				{
					gRobot.walkStatus = waitForBlueTakeBall;
				}
			}
			break;
		}
		default:
			break;
	}
}


/*  ***************************************测试加速度程序*********************************************************** */
void TestParaMotor(void)
{
	#define VEL (3000.0f)
	#define DIRECTION (0.0f)

	static uint8_t testStatus = 0;

	delayCount++;

	delayCount %= 10000;

	switch(testStatus)
	{
		case 0:
		{
			SendCmd2Driver(1000.0f,-60.0f,1000.0f,-60.0f,1000.0f,-60.0f);
			// SetTargetVel(500.0f,180.0f,0.0f);    

			if(delayCount > 50)
			{
				delayCount = 0;
				testStatus = 1;
			}

			break;
		}
		case 1:
		{
			SendCmd2Driver(0.0f,-60.0f,0.0f,-60.0f,0.0f,-60.0f);
			// SetTargetVel(0.0f,180.0f,0.0f);
	
			break;
		}
		default:
			break;
	}
}

/*  ********************************************Other Functions***************************************************** */
uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	velVector_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.xVel;
	float speedY = actSpeed.yVel;
	
	float speed = sqrtf(speedX * speedX + speedY * speedY);
	
	if(speed>speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



/**
  * @brief	用车的垂直方向的加速度计算平衡角度
  * @name	AccBlanceAngle
  * @note	None
  * @param	gyroAcc
  * @retval	None
  */
void AccBlanceAngle(void)
{
	//四个周期一算		gRobot.robotVel,gRobot.velDir,gRobot.omg
	static float frontVel = 0.f;
	static float addVel = 0.f;
	static float addVelErr =0.f;
	static float lastVelOutPut = 0.f;
	static float lastFrontVel = 0.f;
	static float lastAddVel = 0.f;
	static float lastVelDirec = 0.f;
	float trimA = 0.f;
	static float test[10] = {0};
	if (fabs(gRobot.preDictVel - lastVelOutPut) > 2.f)//才进行计算  
	{
		
		//===================================
		test[0] = gRobot.preDictVel * cos((gRobot.preDictVelDirec)/57.3f);	//vel of world
		test[1] = test[0] * cos((90.f- GetAngle())/57.3f);	//world 2 momentum
		test[3] = test[1] -test[2] ;		//addVel
		test[2] = test[1];
		//===================================


		frontVel = gRobot.preDictVel * cos((lastVelDirec + 90.f - gRobot.preDictVelDirec)/57.3f);
		addVel = frontVel - lastFrontVel;
		addVelErr = addVel - lastAddVel;
//		gRobot.balanceAngle = gRobot.balanceAngle + addVel;
		if(fabs(lastAddVel - addVel) * 0.15f > 1.f && fabs(addVel) > 0.0000000000001f 
			 && fabs(gRobot.preDictVel) > 10.f)
		{
			// gRobot.pidBalanceAngleNeedLast = gRobot.momentumStruct.afterVelAct;	//及时更新记录的上一次速度值，防止角度环里也满足条件变期望
			trimA = addVel * 0.1f;
			if(fabs(addVel) < 2.5f)
			{
				trimA = addVel/fabs(addVel) * 2.5;
			}
			if(fabs(addVel) > 4.2f)
			{
				trimA = addVel/fabs(addVel) * 4.2f;
			}
			// gRobot.pidBalanceAngle = gRobot.pidBalanceAngle + trimA;
//			gRobot.pidBalanceAngle = gRobot.pidBalanceAngle + addVel/fabs(addVel)*2.6f;//根据加速度变化率，预测调节平衡角
		}
		lastAddVel = addVel;
		lastFrontVel = frontVel;
	}
	if (fabs(gRobot.velDir) < 0.01f)
	{
		lastVelDirec = GetAngle();
	}
	else
	{
		lastVelDirec = gRobot.preDictVelDirec;
	}
	lastVelOutPut = gRobot.preDictVel;
	//判断角速度连续增大,速度连续增大则认为平衡角度改变
	// gRobot.expectCtrlT[0] = frontVel;
	// gRobot.expectCtrlT[1] = addVel* 0.15f;
	// gRobot.expectCtrlT[2] = addVelErr* 0.15f;
	// gRobot.expectCtrlT[3] = lastVelOutPut;
	// gRobot.expectCtrlT[4] = gRobot.preDictVel;
	// gRobot.expectCtrlT[5] = trimA;

	// gRobot.expectCtrlT[6] = test[0];
	// gRobot.expectCtrlT[7] = test[1];
	// gRobot.expectCtrlT[8] = GetAngle();
	// gRobot.expectCtrlT[9] = test[3];
	// if (gRobot.balanceAngle > 5.f)
	// {
	// 	gRobot.balanceAngle = 5.f;
	// }
	// else if (gRobot.balanceAngle < -7.f)
	// {
	// 	gRobot.balanceAngle = -7.f;
	// }
	
	// if (gRobot.pidBalanceAngle > 5.f)
	// {
	// 	gRobot.pidBalanceAngle = 5.f;
	// }
	// else if (gRobot.pidBalanceAngle < -10.f)
	// {
	// 	gRobot.pidBalanceAngle = -10.f;
	// }
}

/**
* @name   LimitIncludeAngle
* @brief  限制夹角在+1~ -1之间,并计算出结果
* @param  angleOne 角度one
* @param  angleTwo 角度two
* @retval 返回正确夹角
*/
float LimitIncludeAngle(float angleOne,float angleTwo)
{
	float cosIn = 0.f;
	float IncludegAngle  = 0.f;
	cosIn = cosf(angleOne) * cosf(angleTwo) + sinf(angleOne) * sinf(angleTwo);
	if (cosIn > 0.9999)
	{
		cosIn = 0.9999f;
	}
	if (cosIn < -0.9999)
	{
		cosIn = -0.9999f;
	}
	IncludegAngle = acosf(cosIn);
	return IncludegAngle;
}

/**
  * @brief	jiao速度输出六个周期的平均值
  * @name	Get_AverOmg
  * @note	None
  * @param	omg
  * @retval	六个周期的平均值
  */
float Get_AverOmg(float omg)
{
	const uint16_t FilterOrder = 40;
	static float array[40] = {0};
	static float sum = 0.f;
	static float avg = 0.f;
	static float old = 0.f;
	static uint16_t pos = 0;

	if(pos > 39)
	{
		pos = 39;
	}
	
	old = array[pos];
	
	array[pos] = omg;
		
	sum = (sum - old) + array[pos];
		
	avg = sum / FilterOrder;

	pos = (pos+1) % FilterOrder;
	
	return avg;
}

/**
  * @brief	速度输出六个周期的平均值
  * @name	Get_AverVel
  * @note	None
  * @param	vel
  * @retval	六个周期的平均值
  */
float Get_AverVel(float vel)
{
	const uint16_t FilterOrder = 40;
	static float array[40] = {0};
	static float sum = 0.f;
	static float avg = 0.f;
	static float old = 0.f;
	static uint16_t pos = 0;

	if(pos > 39)
	{
		pos = 39;
	}
	
	old = array[pos];
	
	array[pos] = vel;
		
	sum = (sum - old) + array[pos];
		
	avg = sum / FilterOrder;

	pos = (pos+1) % FilterOrder;
	
	return avg;
}


/**
  * @brief	速度方向输出六个周期的平均值
  * @name	Get_AverVelDirec
  * @note	None
  * @param	vel
  * @retval	六个周期的平均值
  */
float Get_AverVelDirec(float velDirec)
{
	const uint16_t FilterOrder = 40;
	static float array[40] = {0};
	static float sum = 0.f;
	static float avg = 0.f;
	static float old = 0.f;
	static uint16_t pos = 0;

	if(pos > 39)
	{
		pos = 39;
	}
	
	old = array[pos];
	
	array[pos] = velDirec;
		
	sum = (sum - old) + array[pos];
		
	avg = sum / FilterOrder;

	pos = (pos+1) % FilterOrder;

	// for(int i = 0; i < FilterOrder;i++)
	// {
	// 	fprintf(fpWrite,"D%f",array[i]);
	// }
	
	return avg;
}

