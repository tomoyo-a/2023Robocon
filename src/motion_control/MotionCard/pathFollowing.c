/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   
*Author：      
*Date：       
*Description： 
*
*Version：     V1.0
********************************************************************/
#include <math.h>
#include "pathFollowing.h"
#include "ringbuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "MotionCard.h"
#include "Move.h"
#include "stdint.h"
#include "robot.h"
#include "SpeedPlaning.h"
#include "pps.h"
#include "path.h"

extern robotVel_t targetVel;
extern KeyPointInf_t *ringBuffer;

#define EXPERT_PID
PathfollowPara_t goForFirstPara = 
{
	{2.9,0,0,2.0},//0.15
	{3.8,0,0,2.9},//0.2
	{4.0,4.5,0,3.25},//0.25
	{4.5,0,0,0},//3.0
	{4.8,5.2,0,0},//3.5
	{5.3,5.9,2.4,3.9},//4
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0},
	{0,0,0,0}
};
/*********************************************************************************
* @name 	PathFollowingNew
* @brief	路径跟随函数
* @param	percent 速度的百分比，若为1代表100%所规划的速度运行。范围为大于0,如果超过1，会超过机器人承受速度
* @retval	无
**********************************************************************************/
//调节量：按当前点到虚拟位置点作为偏差 提前30m到终点不调
//降速的量：TR第一段直线：垂直于速度方向的误差作为偏移量降速
//提前量：TR第一段直线:会减去 垂直于速度方向的误差 这个偏移量
int PathFollowing(float percent)
{
	static float vell = 150.0f;
	float velDir = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngleVP = 0.0f;
	float posAngleVT = 0.0f;
	float robotlen = 0.0f;
	float disRealPos2VirTarget = 0.0f;
	float disRealPos2VirPos = 0.0f;
	robotVel_t adjustVel = {0.0f};
	PointU_t virtualPos,virtualTarget;
	//计算预测的相距150mm的点
	float predictRobotLen = 0.f;
	PointU_t predictPos;
	float predictVel = 0.f;

	if(percent < 0.0f || percent > 1.2f)
	{
		printf("Invalid parameter\n");
		return -1;
	}

	//当前点与虚拟位置点距离 和 虚拟位置点到虚拟目标点距离之和
	float VIEW_L = 0.0f;
	//提前量
	switch(gRobot.walkStatus)
	{
		case goForBlueTakeBall:
		{
			if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex - 2].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 7.0f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*7.8f;//走圆弧增大提前量
			}
			else if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex - 2].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 4.5f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*6.5f;
			}
			else
			{
				VIEW_L = GetPosPresent().vel/50.0f*6.0f;
			}
			if(GetY()>3300.0f)//走圆弧，增大提前量
			{
				VIEW_L = GetPosPresent().vel/50.0f*7.8f;
			}
			// else if(GetY()>3000.0f)//走圆弧，增大提前量
			// {
			// 	VIEW_L = GetPosPresent().vel/50.0f*6.0f;
			// }
			break;
		}
		case goForRedTakeBall:
		{
			if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex - 2].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 7.0f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*8.5f;//走圆弧增大提前量
			}
			else if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex - 2].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 4.5f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*7.5f;
			}
			else
			{
				VIEW_L = GetPosPresent().vel/50.0f*6.3f;
			}
			if(GetY() > 750.f && GetY() < 1700.f)//走圆弧，增大提前量
			{
				VIEW_L = GetPosPresent().vel/50.0f*8.1f;
			}
			if(GetY()>4100.0f)//走圆弧，增大提前量
			{
				VIEW_L = GetPosPresent().vel/50.0f*9.4f;
			}
			else if(GetY()>3000.0f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*10.7f;
			}
			else if(GetY() > 2300.f && GetY() < 2700.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*7.1f;
			}
			else if(GetY()>1700.0f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*8.8f;
			}
			break;
		}
		case goForRedTakingBall:
		case goForBlueTakingBall:
		{
			VIEW_L = GetPosPresent().vel/50.0f*5.5f;
			break;
		}
		case goForRedPut1st:
		{
			if(GetY() < 1200.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*10.f;
			}
			else if(GetY() < 1500.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*9.f;
			}
			else if(GetY() < 2200.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*10.f;
			}
			else if(GetY() < 3600.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*9.6f;
			}
			else
			{
				VIEW_L = GetPosPresent().vel/50.0f*9.f;
			}
			break;
		}
		case goForBluePut1st:
		{
			if(gRobot.cvPeriodFlag == 1 && GetY() > 1500.f)
			{
				VIEW_L = GetPosPresent().vel/50.0f*10.f;
			}
			else
			{
				VIEW_L = GetPosPresent().vel/50.0f*5.5f;
			}
			
			break;
		}
		case goForRedPut2nd:
		case goForBluePut2nd:
		{
			VIEW_L = GetPosPresent().vel/50.0f*5.0f;
			break;
		}
		default:
		{
			VIEW_L = GetPosPresent().vel/50.0f*2.0f;
			break;
		}
	}

	//获取定位系统所计算的机器人实际行走路径长度
	robotlen = GetPath();
	
	gRobot.debugInfomation.robotlenVP = robotlen;

	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);
	
	//计算当前点到虚拟位置点的距离(直线距离)	
	disRealPos2VirPos = CalculatePoint2PointDistance(GetPosPresent().point,virtualPos.point);

	if(VIEW_L - disRealPos2VirPos >= 0.0f)
	{
		//加上 提前量与偏移量的差值
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos;
	}
	
	gRobot.debugInfomation.robotlenVT = robotlen;
	
	//求取虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);

	//求取预测点
	float PRE_VIEW_L = GetPosPresent().vel/100.0f*20.f;
	gRobot.debugInfomation.predictVel_L = PRE_VIEW_L;
	predictRobotLen = GetPath() + PRE_VIEW_L;
	predictPos = SerchVirtualPoint2(predictRobotLen);
	
	//计算实际位置距离虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(GetPosPresent().point,virtualTarget.point);

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	
	if(GetPath() < GetLength())
	{
		if(disAdd > 0)
		{
			AddPath(2*disAdd);
		}
	}
	else
	{
		//当记录的路程大于轨迹总路程后停止记录路程
		UpdateLenStop();
	}
	
	//虚拟位置点姿态角
	posAngleVP = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1))*virtualPos.u);
	
	//两端点之间角度的插值
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(virtualTarget.endPtr) , GetRingBufferPointPoseAngle(virtualTarget.startPtr));
	
	//虚拟目标点姿态角
	posAngleVT = CalculateAngleAdd(GetRingBufferPointPoseAngle(virtualTarget.startPtr),angleErr*virtualTarget.u);
	//角速度
	switch(gRobot.walkStatus)
	{
		case goForBlueTakeBall:
		{
			float aimAngle = 0.f;

			//红场出发区那段不要沿着速度方向走
			// if(gRobot.cvData.filed == RED_COURT && GetY() < 750.f)
			// {
			// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,5.5f,45.f);
			// }
			// else
			{
				if(gRobot.planData.NowposIndex >= 19)
				{
					angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.5f,120.f);
				}
				else
				{
					aimAngle = virtualTarget.direction - 180.f;
					AngleLimit(&aimAngle);
					angularVel = AngleControl(GetPosPresent().direction,aimAngle,4.0f,75.f);
				}	
			}
			// if(GetX() < 1200.0f && GetY() > 11350.0f && GetX() >= 748.0f)
			// {
			// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,1.5f,1.3f);
			// }	
			// else if(GetX() < 748.0f && GetY() > 11350.0f)
			// {
			// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.1f,2.0f);
			// }
			break;
		}
		//红场取球
		case goForRedTakeBall:
		{
			float aimAngle = 0.f;
			// if(GetY() > 3650.f)
			{
				angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.2f,80.f);
				// if(fabs(angularVel) > 40.f)
				// {
				// 	angularVel = fabs(angularVel)/angularVel*40.f; 
				// }
			}
			break;
		}
		case goForRedTakingBall:
		case goForBlueTakingBall:
		{
			// angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.0f,2.3f);
			float aimAngle = 0.f;
			// aimAngle = -90.f + AngleAdjustByStickPos(gRobot.stickLPos, gRobot.stickRPos);
			
			aimAngle = -90.f;
			if(judgeTakingBallVel(gRobot.robotVel) > 3)
			{
				aimAngle = -90.f;
			}
			gRobot.debugInfomation.takingAngle = aimAngle;
			angularVel = AngleControl(GetPosPresent().direction, aimAngle, 35.f, 60.f);
			
			break;
		}
		case goForRedPut1st:
		{
			float aimAngle = 0.f;
			// if(gRobot.planData.NowposIndex >= 16 && gRobot.planData.NowposIndex <= 18)
			// {
			// 	aimAngle = virtualTarget.direction - 180.f;
			// 	AngleLimit(&aimAngle);
			// 	angularVel = AngleControl(GetPosPresent().direction,aimAngle,6.0f,25.f);
			// }
			// else
			if(GetY() < 4290.f && GetY() >= 4100.f)
			{
				angularVel = AngleControl(GetPosPresent().direction,50.f,4.f,25.f);
			}
			else if(GetY() < 1200.f)
			{
				angularVel = AngleControl(GetPosPresent().direction,90.f,4.f,50.f);
			}
			else if(GetY() < 4100.f)
			{
				angularVel = AngleControl(GetPosPresent().direction,90.f,5.f,100.f);//85
			}
			else 
			{
				angularVel = AngleControl(GetPosPresent().direction,-90.f,5.f,50.f);
			}
			break;
		}
		case goForBluePut1st:
		{
			float aimAngle = 0.f;
			// if(gRobot.planData.NowposIndex >= 16 && gRobot.planData.NowposIndex <= 18)
			// {
			// 	aimAngle = virtualTarget.direction - 180.f;
			// 	AngleLimit(&aimAngle);
			// 	angularVel = AngleControl(GetPosPresent().direction,aimAngle,6.0f,25.f);
			// }
			// else
			{
				angularVel = AngleControl(GetPosPresent().direction,posAngleVT,5.f,60.f);
			}
			break;
		}
		case goForRedPut2nd:
		case goForRedPut3rd:
		case goForRedPut4th: 
		case goForRedPut5th:
		case goForBluePut2nd:
		case goForBluePut3rd:
		case goForBluePut4th: 
		case goForBluePut5th:
		{
			angularVel = AngleControl(GetPosPresent().direction,90.f,5.5f,40.f);
			break;
		}
		default:
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.35f,2.15f);
			break;
		}
	}
		
	//目标速度方向
	velDir = virtualTarget.direction;
	
	AngleLimit(&velDir);
	
	//目标速度
	vell = GetRingBufferPointVell(virtualTarget.startPtr)+(GetRingBufferPointVell(virtualTarget.endPtr) - GetRingBufferPointVell(virtualTarget.startPtr))*virtualTarget.u;

	vell = vell*percent;

	//预测点的速度
	predictVel = GetRingBufferPointVell(predictPos.startPtr)+(GetRingBufferPointVell(predictPos.endPtr) - GetRingBufferPointVell(predictPos.startPtr))*predictPos.u;
	predictVel = predictVel*percent;
	
	
	gRobot.debugInfomation.originVel = vell;
	gRobot.debugInfomation.originVelDir = velDir;
	

	gRobot.debugInfomation.fixedVel = vell;
	//计算当前点到目标点的位置调节量
	adjustVel = GetAdjustVel(GetPosPresent().point,virtualPos,vell);

	AdjustVel(&vell,&velDir,adjustVel);

	SetTargetVel(vell,velDir,angularVel);
		
	gRobot.debugInfomation.VIEW_L = VIEW_L;								//提前量
	gRobot.debugInfomation.virtualPos = virtualPos;						//虚拟位置点
	gRobot.debugInfomation.posAngleVP = posAngleVP;						//虚拟位置点姿态角
	gRobot.debugInfomation.virtualTarget = virtualTarget;				//虚拟目标点
	gRobot.debugInfomation.posAngleVT = posAngleVT;						//虚拟目标点姿态角
	gRobot.debugInfomation.disRealPos2VirPos = disRealPos2VirPos;		//点到虚拟位置点距离
	gRobot.debugInfomation.disRealPos2VirTarget = disRealPos2VirTarget;	//点到虚拟目标点距离
	gRobot.debugInfomation.disAdd = disAdd;								//修正距离
	gRobot.debugInfomation.adjustVel = adjustVel;						//调节速度
	gRobot.debugInfomation.sumVel = vell;								//目标速度
	gRobot.debugInfomation.sumVelDir = velDir;							//目标速度方向
	gRobot.debugInfomation.omega = angularVel;							//目标角速度

	gRobot.debugInfomation.predictLen = predictRobotLen;                //预测点的轨迹长度
	gRobot.debugInfomation.predictPos = predictPos;                     //预测点
	gRobot.debugInfomation.predictVel = predictVel;                     //预测点的速度
	gRobot.debugInfomation.predict_U  = predictPos.u;
	
	return 1;
}




/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param  angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget,float kp,float kd)
{
	/****************************普通PD控制*******************************/
	
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	static float lastAngleTarget = 0.f;
	float dTerm = 0.0f,dTermFliter = 0.0f;
	float ki = 0.15f;//增加积分项
	static float errSum = 0.f;
	//PD控制器
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);

	dTerm = (angleErr - lastAngleErr);
	//低通滤波
	dTermFliter = 0.5f*dTerm + 0.5f*lastAngledTerm;

	if (gRobot.walkStatus == goForRedTakingBall)
	{
		ki = 0.f;
		// ki = 0.18f;
		// if (fabs(lastAngleTarget - angleTarget) > 1.f)
		// {
		// 	errSum = 0.f;
		// 	dTermFliter = 0.f;
		// }
	}

	if(fabs(angleErr) > 0.1f && fabs(angleErr) < 5.f && gRobot.gamestart == 1)
	{
		errSum += angleErr;
	}
	else
	{
		errSum = 0.f;
	}
	
	// if((gRobot.walkStatus>=goForRedPut2nd && gRobot.walkStatus <=  goForRedPut5th)||
	// (gRobot.walkStatus>=goForBluePut2nd && gRobot.walkStatus <=  goForBluePut5th))	//放球时有稳态误差引入积分项
	{		
		angularVel = angleErr * kp + dTermFliter * kd + errSum * ki;
	}
	// else
	// {
	// 	angularVel = angleErr * kp + dTermFliter * kd;
	// }
	
	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;
	// gRobot.debugInfomation.omega = angularVel;
	gRobot.debugInfomation.angleAim = angleTarget;
	gRobot.debugInfomation.angleKp = angleErr * kp;
	gRobot.debugInfomation.angleKd = dTermFliter * kd;
	gRobot.debugInfomation.errSum = errSum * ki;
	
	angularVelErr = angularVel - GetWZ();

	// gRobot.debugInfomation.omega = angularVel;
	
	if(gRobot.walkStatus == goForBlueTakeBall) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.f;
	}
	else if(gRobot.walkStatus == goForBlueTakingBall) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForBluePut1st) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForBluePut2nd) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.2f;
	}
	else if(gRobot.walkStatus == goForRedPut1st) //增加角速度闭环
	{
		angularVel = angularVel + angularVelErr *0.29f;
	}
	else
	{
		angularVel = angularVel + angularVelErr *0.0f;
	}

	// if(gRobot.walkStatus == goForBlueTakeBall && GetY() > 2800.f)
	// {
	// 	//限幅
	// 	if(angularVel>90.0f)
	// 	{
	// 		angularVel = 90.0f;
	// 	}
	// 	else if(angularVel<-90.0f)
	// 	{
	// 		angularVel = -90.0f;
	// 	}
	// }

	//限幅
	if(angularVel>235.0f)
	{
		angularVel = 235.0f;
	}
	else if(angularVel<-235.0f)
	{
		angularVel = -235.0f;
	}

	if(gRobot.walkStatus == goForRedPut1st && fabs(angleErr) > 20.f)
	{
		if(angularVel>75.0f)
		{
			angularVel = 75;
		}
		else if(angularVel<-75.0f)
		{
			angularVel = -75.0f;
		}
	}
	// if (gRobot.walkStatus == goForRedTakingBall || gRobot.walkStatus == goForBlueTakingBall)
	// {
	// 	if(angularVel>50.0f)
	// 	{
	// 		angularVel = 50.f;
	// 	}
	// 	else if(angularVel<-50.0f)
	// 	{
	// 		angularVel = -50.0f;
	// 	}
	// }
	if(sqrt(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY()) < 200.0f \
	&& gRobot.walkStatus != goForRedPut1st && gRobot.walkStatus != goForBluePut1st\
	&& gRobot.walkStatus != goForRedTakingBall && gRobot.walkStatus != goForBlueTakingBall)
	{
		if(angularVel>30.0f)
		{
			angularVel = 30.0f;
		}
		else if(angularVel<-30.0f)
		{
			angularVel = -30.0f;
		}
	}
	lastAngleTarget = angleTarget;
	gRobot.debugInfomation.angleVelOUtput = angularVelErr;
	gRobot.debugInfomation.outputOmega = angularVel;
		
	return angularVel;
}

/*********************************************************************************
* @name 	PostionControl
* @brief	位置闭环控制程序
* @param	distance 当前的位置误差 单位 mm
* @param  	kp 专家PID参数
* @param  	kd 专家PID参数

* @retval	无
**********************************************************************************/
float PostionControl(float distance, float kp, float kd)
{
	/********************************专家PID****************************
	 *    5条规则
	 * 1. 误差绝对值很大，按照最大（绝对值）输出
	 * 2. 误差×delta误差>0，误差增大，如果误差值大于中值，则加大控制力度，否则一般控制
	 * 3. 误差×delta误差<0 && delta误差×delta（上一次误差）>0，误差减小 ，一般控制
	 * 4. 误差×delta误差<0 && delta误差×delta（上一次误差）<0，处于极值 ，一般控制（保持输出不变即可）
	 * 5. 误差很小，一般控制（可能是系统静差）
	 */

	static ExpertPID_t distancePID;
  	float adjustVelOutput = 0.0f, adjustVelOutputErr = 0.0f;//本次调节输出值
	
	distancePID.maximum = 1500.f;//最大输出速度 待修正
	distancePID.minimum = 0.f;//待修正
	distancePID.errabsmax = 400.f;// 最大距离误差 
	distancePID.errabsmid = 225.f;//中等距离误差 
	distancePID.errabsmin =  46.f;//小距离误差   

	distancePID.kp = kp;
	distancePID.kd = kd;
	distancePID.thisErr = distance;//本次偏差
	distancePID.dErr = distancePID.thisErr - distancePID.lastErr;//偏差微分

	if(fabs(distancePID.thisErr) > distancePID.errabsmax)//误差太大，执行规则1
	{
		// printf("11111\r\n");
		distancePID.result = fabs(distancePID.thisErr) / distancePID.thisErr * distancePID.maximum;//最大输出
	}	
	else if((distancePID.thisErr * distancePID.dErr > 0)||(fabs(distancePID.dErr) < 0.1f))//误差增大，执行规则2
	{
		// printf("222222\r\n");
		if(fabs(distancePID.thisErr) >= distancePID.errabsmid)//误差较大,加大控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
		else//误差较小，一般控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
	}
 	else if(((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr > 0)) || (distancePID.thisErr == 0))//误差减小，执行规则3，一般控制
  	{
		// printf("333333\r\n");
    	distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
  	}
	else if((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr < 0))//误差极值，执行规则4
  	{
		// printf("444444\r\n");
    	if(abs(distancePID.thisErr) >= distancePID.errabsmid) //误差较大，则较强控制
		{
			distancePID.result = 1.0* distancePID.kp * distancePID.thisErr;
		}
		else//误差一般大，一般控制
		{
			distancePID.result = 1.0 * distancePID.kp * distancePID.thisErr;
		}
  	}
	else if((fabs(distancePID.thisErr) <= distancePID.errabsmin) && (abs(distancePID.thisErr)>0))//误差很小，执行规则5
	{
		// printf("555555\r\n");
		distancePID.result = 1.0 * distancePID.kp * distancePID.dErr + 0.8 * distancePID.ki * distancePID.thisErr;//可能存在稳态误差，添加ki控制
	}
	distancePID.preErr = distancePID.lastErr;
	distancePID.lastErr = distancePID.thisErr;
	distancePID.lastdErr = distancePID.dErr;

	return distancePID.result;

	printf("De %d %d %d %d ",(int)distancePID.thisErr, (int)distancePID.dErr,\
		(int)distancePID.lastdErr, (int)distancePID.result);
	/********************************专家PID***************************/
}

robotVel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell)
{
	#define MAX_ADJUST_VEL (800.0f)
	robotVel_t adjustVel = {0};
	float distance = 0.0f;
	float angle = 0.0f;

	angle = CalculateLineAngle(robotPos,adjustTarget.point);
	
	distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);
	if(gRobot.walkStatus == goForRedTakingBall || gRobot.walkStatus == goForBlueTakingBall)
	{
		distance = 0.f;
	}
	
	//死区
	// if(distance<5.0f)
	// {
	// 	distance = 0.0f;
	// }
	// else
	// {
	// 	distance-=5.0f;
	// }
	
	if(distance<=0.0f)
	{
		distance = 0.0f;
	}
	gRobot.debugInfomation.distance = distance; 
	
	//计算调节速度大小和方向

	if(distance <= 20.0f)
	{
		distance = distance * 1.5f;
//		adjustVel.vel = distance * 7.5f;
	}
	else if(distance <= 40.0f) 
	{
		distance = distance * 2.8f - 26.0f;//2.5-20
	}
	else if(distance <= 60.0f)
	{
		distance = distance * 2.2f - 2.0f;//3.3  -52
	}
	else if(distance <= 100.0f)
	{
		distance = distance * 2.5 - 20.0f;//4.0  -94//4.5 -124//6.0 -214
	}
	else if(distance <= 200.0f)
	{
		distance = distance * 3.0f -70.0f;//4.5  -144//4.5 -124
//		adjustVel.vel = distance * 4.5f + 120.0f;
	}
	else
	{
		distance = distance * 3.0f;
//		adjustVel.vel = distance * 4.0f + 320.0f;
	}
	gRobot.debugInfomation.adjustDistance = distance;
	switch(gRobot.walkStatus)
	{
		case goForBlueTakeBall:
		{
			if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 7.5f)
			{
				adjustVel.vel = distance * 5.2f;//1.0//1.2
			}
			else if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 2.5f)
			{
				adjustVel.vel = distance * 5.2f;//1.0//1.2
			}
			else
			{
				adjustVel.vel = distance * 4.8f;//1.0//1.2
			}
			if(GetY() > 3900.f)
			{
				adjustVel.vel = distance * 4.8f;//1.0//1.2
			}
			if(fabs(gRobot.cvData.rectifyErr.x) > 0.f)
			{
				adjustVel.vel = adjustVel.vel * 0.8f;
			}
			
			// else if(GetY() > 3600.f)
			// {
			// 	adjustVel.vel = distance * 3.7f;//1.0//1.2
			// }
			// else
			// {
			// 	adjustVel.vel = distance * 3.0f;//1.0//1.2
			// }
			break;
		}
		case goForRedTakeBall:
		{
			if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 7.f)
			{
				adjustVel.vel = distance * 4.2f;//1.0//1.2
			}
			else if (LimitIncludeAngle(ringBuffer[gRobot.planData.NowposIndex].angle/57.3f,ringBuffer[gRobot.planData.NowposIndex - 1].angle/57.3f) * 57.3f > 2.5f)
			{
				adjustVel.vel = distance * 4.2f;//1.0//1.2
			}
			else
			{
				adjustVel.vel = distance * 3.6f;//1.0//1.2
			}
			if(GetY() > 500.f && GetY() < 2700.f)
			{
				adjustVel.vel = distance * 4.7f;//1.0//1.2
			}
			if(GetY()>3900.0f)
			{
				adjustVel.vel = distance * 4.35f;//1.0//1.2
			}
			if(fabs(gRobot.cvData.rectifyErr.x) > 0.f)
			{
				adjustVel.vel = adjustVel.vel * 0.8f;
			}
			break;
		}
		case goForRedTakingBall:
		case goForBlueTakingBall:
		{
			adjustVel.vel = distance * 1.0f;
			// if(distance > 100)
			// {
			// 	adjustVel.vel = distance * 1.5f;
			// }
			break;
		}
		case goForRedPut1st:
		{
			if(GetY() < 1200.f)
			{
				adjustVel.vel = distance * 1.5f;
			}
			else if(GetY() < 2200.f)
			{
				adjustVel.vel = distance * 1.5f;
			}
			else if(GetY() < 3500.f)
			{
				adjustVel.vel = distance * 3.8f;
			}
			else
			{
				adjustVel.vel = distance * 3.4f; 
			}
			if(fabs(gRobot.cvData.rectifyErr.x) > 0.f)
			{
				adjustVel.vel = adjustVel.vel * 0.8f;
			}
			break;
		}
		case goForBluePut1st:
		{
			adjustVel.vel = distance * 3.2f;
			if(GetY() < 3000.f)
			{
				adjustVel.vel = distance * 1.0f;
				if (fabs(gRobot.cvData.rectifyErr.x) > 0.f)
				{
					adjustVel.vel = distance * 0.8f;
				}
			}
			break;
		}
		case goForRedPut2nd:
		case goForBluePut2nd:
		{
			adjustVel.vel = distance * 3.2f;
			break;
		}
		default:
		{
			adjustVel.vel = distance * 3.4f;
			break;
		}
	}
	
	
	//规划速度小于0.5m/s 不调节
	//if(vell < 500.f)
	if(vell < 30.f && gRobot.walkStatus == goForBlueTakeBall)
	{
		adjustVel.vel = 0.0f;
	}
	/*else if(vell < 1000.f)
	{
		adjustVel.vel = adjustVel.vel * 0.5f;
//		adjustVel.vel = 0.0f;
	}*/
	
	adjustVel.direction = angle;
	
	//对调节速度大小进行限制
	if(adjustVel.vel>=MAX_ADJUST_VEL)
	{
		adjustVel.vel = MAX_ADJUST_VEL;
	}
	gRobot.debugInfomation.adjustvellast = adjustVel.vel;
	return adjustVel;
}

void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel)
{
	#define ADJUST_KP (1.2f)
	
	float angleErr = 0.0f;
	float projectOnvel = 0.0f;
	vector_t oriVel = {0} , adjust = {0} , result = {0};
	
	oriVel.module = *carVel;
	oriVel.direction = *velAngle;
	
	//计算调节目标速度的大小
	adjust.module = ADJUST_KP * adjustVel.vel;
	
//	if(oriVel.module>(10.0f * adjust.module))
//	{
//		adjust.module = oriVel.module/10.0f;
//	}
	
//	if(adjust.module>*carVel)
//	{
//		adjust.module = *carVel;
//	}
	
	//计算速度方向的调节量并进行限制
	angleErr = adjustVel.direction - *velAngle;
	
	angleErr = angleErr > 180.0f ? angleErr - 360.0f : angleErr; 
	angleErr = angleErr < -180.0f ? 360.0f + angleErr : angleErr;

	angleErr =  ADJUST_KP * angleErr;
	if(angleErr>180.0f)
	{
		angleErr = 180.0f;
	}
	else if(angleErr<-180.0f)
	{
		angleErr = -180.0f;
	}
	
	adjust.direction = *velAngle + angleErr;
	
	adjust.direction = adjust.direction > 180.0f ? adjust.direction - 360.0f : adjust.direction; 
	adjust.direction = adjust.direction < -180.0f ? 360.0f + adjust.direction : adjust.direction;
	
	
	//计算调节量在原速度上的投影大小
	// projectOnvel = CalculateVectorProject(adjust,oriVel);
	// //根据投影大小限制调节后的速度方向
	// if(projectOnvel<=-oriVel.module)
	// {
	// 	adjust = CalculateVectorFromProject(-oriVel.module,adjust.direction,oriVel.direction);
	// }
	
	result = CalculateVectorAdd(oriVel , adjust);
	//对调整结果的速度大小进行限制
	if(result.module>=GetVelMax())
	{
		result.module=GetVelMax();
	}
	
	*carVel = result.module;
	*velAngle = result.direction;
}


