#ifndef _PATHFOLLOWING_H
#define _PATHFOLLOWING_H
#include "calculate.h"
#include "moveBase.h"

typedef struct
{
    float setpoint;   	  	/*设定值*/
    float kp;             	/*比例系数*/
    float ki;          		/*积分系数*/
    float kd;             	/*微分系数*/
    float thisErr;          /*偏差*/
    float lastErr;        	/*前一拍偏差*/
    float preErr;           /*前两拍偏差*/
    float dErr;             /*偏差差值（微分）*/ 
    float lastdErr;         /*上一次偏差差值*/ 
    float result;           /*PID控制器结果*/
    float maximum;          /*输出值上限*/
    float minimum;          /*输出值下限*/
    float errabsmax;        /*偏差绝对值最大值*/
    float errabsmid;        /*偏差绝对值中位值*/
    float errabsmin;        /*偏差绝对值最小值*/
}ExpertPID_t;

typedef struct
{
    float angleKp;
    float angleKd;
    float posKp;
    float posKd;
}oneVelPathfollowPara_t;


typedef struct
{
    oneVelPathfollowPara_t onePara;
    oneVelPathfollowPara_t twoPara;
    oneVelPathfollowPara_t thrPara;
    oneVelPathfollowPara_t fourPara;
    oneVelPathfollowPara_t fivePara;
    oneVelPathfollowPara_t sixPara;
    oneVelPathfollowPara_t sevenPara;
    oneVelPathfollowPara_t eightPara;
    oneVelPathfollowPara_t ninePara;
    oneVelPathfollowPara_t tenPara;
    
}PathfollowPara_t;

float AngleControl(float anglePresent,float velDir,float kp,float kd);
float PostionControl(float distance, float kp, float kd);
robotVel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell);

void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel);


#endif
