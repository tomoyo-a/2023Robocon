#include "balance.h"
#include "robot.h"
#include "pps.h"
#include "moveBase.h"
#include "cv.h"


/**
  * @brief	用视觉识别到的角度和距离计算矫正量,会不会出现矫正量过大的情况？？
  * @name	CvRectifyPos
  * @note	None
  * @param	relativeDis 视觉的相对距离
  * @param	relativeAngle 视觉的偏角
  * @param  retifyType 视觉识别的东西,分别为中间的墙、球、桶，现在三种并无区别
  * @retval	None
  */
Point_t CvRectifyPos(float relativeDis,float relativeAngle, uint8_t retifyType)
{
    static uint8_t lastTime = 0;
    Point_t aimObjectActPos;//目标矫正物的理论位置
    Point_t errPos;//需要补偿的坐标值
    Point_t aimObjectCvPos;//目标矫正物的世界坐标系下的矫正坐标
    float aimObjectAngle = 0.f;//目标矫正物的世界坐标系角度
    Point_t rollErrPos;
    static float lastErrX = 0.f;
    static float lastErrY = 0.f;
    static float rectifyRemain = 0.f;
    float rollToXAngle = 0.f;
    static Point_t grossRecErr;
    static int xRecCnt = 0;//用来记录在x方向还需要矫正多少次7mm
    static int yRecCnt = 0;//用来记录在y方向还需要矫正多少次7mm
    static uint8_t firstColor = NONE;//记录第一个球、桶的颜色，保证矫正的时候对应的是正确的球、桶
    static uint8_t lastAimObject = 0;
    static float calPosX[2] = {0.f};//视觉更新周期时本周期（1）的坐标和上一个周期（0）的坐标
    static float calPosY[2] = {0.f};
    static float calPoseAngle[2] = {0.f};
    static float calPitchAngle[2] = {0.f};
    static uint8_t blackFirst = 0;
    static uint8_t blackFirstCnt = 0;
    static uint8_t cvPeriodCnt = 0;

    //记录视觉识别到第一个球是否是黑色球
    if(gRobot.cvData.colorFlag == 1 && gRobot.cvData.ballOrBucket == BALL && blackFirstCnt < BLACK_BALL_PER_TIME)
    {
        if(gRobot.cvData.colorMsg[0] == BLACK)
        {
            if (lastTime != gRobot.cvData.cvPeriod)
            {
                blackFirstCnt++;
            }
        }
    }
    /*else if(gRobot.cvData.ballOrBucket != BALL)
    {
        blackFirst = 0;
    }*/

    //判断是否第一个球连续多次识别为黑球
    if (blackFirstCnt == BLACK_BALL_PER_TIME && gRobot.cvData.ballOrBucket == BALL)
    {
        blackFirst = 1;
    }
    else if(gRobot.cvData.ballOrBucket != BALL)
    {
        blackFirst = 0;
    }

    //更新用于计算坐标矫正的坐标（使用视觉上一周期时的坐标来矫正）
    if (lastTime != gRobot.cvData.cvPeriod)
    {
        calPosX[0] = calPosX[1];
        calPosY[0] = calPosY[1];
        calPoseAngle[0] = calPoseAngle[1];
        calPitchAngle[0] = calPitchAngle[1];
        calPosX[1] = GetCorrectX();
        calPosY[1]= GetCorrectY();
        calPoseAngle[1] = GetAngle();
        calPitchAngle[1] = GetPitchAngle();
    }

    //记录第一个球、桶的颜色
    if(lastAimObject != retifyType)//球、桶切换的时候清0
    {
        firstColor = NONE;
    }
    if(gRobot.cvData.colorFlag == 1 && firstColor == NONE)
    {
        firstColor = gRobot.cvData.colorMsg[0];
    }

    if(lastTime != gRobot.cvData.cvPeriod && gRobot.cvData.locateFlag == 1 \
    && (firstColor == gRobot.cvData.colorMsg[0] || gRobot.cvData.colorFlag == 0))
    {
        //为不同目标矫正物坐标赋值
        switch(retifyType)
        {
            case BAFFLE_TYPE://中间的墙
            {
                aimObjectActPos.x = 1000.f*COUTR_COLOR_X;
                aimObjectActPos.y = 2550.f;
                break;
            }
            case LEFT_WALL_TYPE://侧边的墙
            {
                aimObjectActPos.x = 0.f;
                aimObjectActPos.y = calPosY[0];
                break;
            }
            case BALL_TYPE://第一个球
            {
                if(gRobot.cvData.filed == BLUE_COURT)
                {
                    if(blackFirst == 0)
                    {
                        aimObjectActPos.x = 250.f;
                        aimObjectActPos.y = 4700.f;
                    }
                    else
                    {
                        aimObjectActPos.x = 625.f;
                        aimObjectActPos.y = 4700.f;
                    }
                }
                else
                {
                    if(blackFirst == 0)
                    {
                        aimObjectActPos.x = -1750.f;
                        aimObjectActPos.y = 4700.f;
                    }
                    else
                    {
                        aimObjectActPos.x = -1375.f;
                        aimObjectActPos.y = 4700.f;
                    }
                }
                break;
            }
            case BUCKET_TYPE://第一个桶（需要区分红蓝场）
            {
                if(gRobot.cvData.filed == BLUE_COURT)//蓝场
                {
                    aimObjectActPos.x = 750.f;
                    aimObjectActPos.y = 400.f;
                }
                else if(gRobot.cvData.filed == RED_COURT)//红场，先取最边上的球
                {
                    aimObjectActPos.x = -1750.f;
                    aimObjectActPos.y = 400.f;
                }
                break;
            }
            default:
            {
                aimObjectActPos.x = 0.f;
                aimObjectActPos.y = calPosY[0];
                break;
            }
        }

        //计算目标矫正物的世界坐标系姿态角
        aimObjectAngle = calPoseAngle[0] + relativeAngle + 180.f;
        AngleLimit(&aimObjectAngle);
        //计算横滚角带来的摄像头的坐标变化
        rollErrPos.x = L515_HEIGHT*sinf(calPitchAngle[0]/57.3f);
        //计算摄像头的z轴到地面的投影与世界坐标系下x轴的夹角
        if(gRobot.walkStatus == goForBlueTakeBall || gRobot.walkStatus == goForRedTakeBall)
        {
            rollToXAngle = calPoseAngle[0] + 90.f;
        }
        else if(gRobot.walkStatus == goForBluePut1st || gRobot.walkStatus == goForRedPut1st)
        {
            rollToXAngle = calPoseAngle[0] - 90.f;
        }
        AngleLimit(&rollToXAngle);
        //计算目标物的世界坐标系坐标
        //横滚角带来的x偏差需要区分去或来的路上
        if(gRobot.walkStatus == goForBlueTakeBall || gRobot.walkStatus == goForRedTakeBall)//去取球
        {
            aimObjectCvPos.x = calPosX[0] + relativeDis*cosf(aimObjectAngle/57.3f) - rollErrPos.x*cosf(rollToXAngle/57.3f);
        }
        else //去放球
        {
            aimObjectCvPos.x = calPosX[0] + relativeDis*cosf(aimObjectAngle/57.3f) + rollErrPos.x*cosf(rollToXAngle/57.3f);
        }
        aimObjectCvPos.y = calPosY[0] + relativeDis*sinf(aimObjectAngle/57.3f);

        //连续识别到位置
        if (gRobot.cvData.locateFlag == 1 && cvPeriodCnt < CV_PER_TIME\
        && (gRobot.walkStatus == goForBluePut1st || gRobot.walkStatus == goForRedPut1st))
        {
            if (lastTime != gRobot.cvData.cvPeriod)
            {
                cvPeriodCnt++;
            }
        }   
        if (cvPeriodCnt ==  CV_PER_TIME && gRobot.cvData.locateFlag == 1)
        {
            gRobot.cvPeriodFlag = 1;
        }
        //判断算出来的坐标是否在目标物附近，若不在则认为是误识别
        if((sqrtf(powf(aimObjectActPos.x - aimObjectCvPos.x, 2.f) + powf(aimObjectActPos.y - aimObjectCvPos.y, 2.f)) < 80.f && retifyType == BALL_TYPE)\
        || (((sqrtf(powf(aimObjectActPos.x - aimObjectCvPos.x, 2.f) + powf(aimObjectActPos.y - aimObjectCvPos.y, 2.f)) < 100.f && calPosY[0] < 2800.f)\
        || (sqrtf(powf(aimObjectActPos.x - aimObjectCvPos.x, 2.f) + powf(aimObjectActPos.y - aimObjectCvPos.y, 2.f)) < 70.f && calPosY[0] >= 2800.f)) && retifyType == BUCKET_TYPE && (GetCorrectY() < 2800.f || gRobot.cvPeriodFlag == 1)))
        {
            //若视觉再次识别到了，重新更新矫正量
            xRecCnt = 0;
            yRecCnt = 0;
            //记录视觉给出的总的矫正误差
            grossRecErr.x = aimObjectActPos.x - aimObjectCvPos.x;
            grossRecErr.y = aimObjectActPos.y - aimObjectCvPos.y;
            gRobot.debugInfomation.oriGrossRecErr.x = grossRecErr.x;
            gRobot.debugInfomation.oriGrossRecErr.y = grossRecErr.y;
            //计算补偿坐标=视觉目标物坐标-实际目标物坐标
            if(fabs(grossRecErr.x - lastErrX) < MAX_ADJUST_PER_TIME)
            {
                errPos.x = grossRecErr.x;
            }
            else
            {
                grossRecErr.x = grossRecErr.x - lastErrX;
                errPos.x = lastErrX + grossRecErr.x/fabs(grossRecErr.x)*MAX_ADJUST_PER_TIME;
                //记录需要矫正多少次7mm
                xRecCnt = fabs((int)grossRecErr.x) / (int)MAX_ADJUST_PER_TIME;
                xRecCnt -= 1;
            }
            if(fabs(grossRecErr.y - lastErrY) < MAX_ADJUST_PER_TIME)
            {
                errPos.y = grossRecErr.y;
            }
            else
            {
                grossRecErr.y = grossRecErr.y - lastErrY;
                errPos.y = lastErrY + grossRecErr.y/fabs(grossRecErr.y)*MAX_ADJUST_PER_TIME;
                yRecCnt = fabs((int)grossRecErr.y) / (int)MAX_ADJUST_PER_TIME;
                yRecCnt -= 1;
            }
            lastErrX = errPos.x;
            lastErrY = errPos.y;
        }
        else
        {
            if(xRecCnt > 0)
            {
                errPos.x = lastErrX + grossRecErr.x/fabs(grossRecErr.x)*MAX_ADJUST_PER_TIME;
                xRecCnt -= 1;
                lastErrX = errPos.x;
            }
            else
            {
                errPos.x = lastErrX;
            }
            if(yRecCnt > 0)
            {
                errPos.y = lastErrY + grossRecErr.y/fabs(grossRecErr.y)*MAX_ADJUST_PER_TIME;
                yRecCnt -= 1;
                lastErrY = errPos.y;
            }
            else
            {
                errPos.y = lastErrY;
            }
        }
        // printf("aimCV %f %f \n", GetX(), GetY());
        gRobot.debugInfomation.cvDis = relativeDis;
        gRobot.debugInfomation.cvAngle = aimObjectAngle;
        gRobot.debugInfomation.cvWorldPos.x = aimObjectCvPos.x;
        gRobot.debugInfomation.cvWorldPos.y = aimObjectCvPos.y;
        gRobot.debugInfomation.rollErrPosX = rollErrPos.x*cosf(rollToXAngle/57.3f);
    }
    else
    {
        //每个周期矫正7mm
        if(xRecCnt > 0)
        {
            errPos.x = lastErrX + grossRecErr.x/fabs(grossRecErr.x)*MAX_ADJUST_PER_TIME;
            xRecCnt -= 1;
            lastErrX = errPos.x;
        }
        else
        {
            errPos.x = lastErrX;
        }
        if(yRecCnt > 0)
        {
            errPos.y = lastErrY + grossRecErr.y/fabs(grossRecErr.y)*MAX_ADJUST_PER_TIME;
            yRecCnt -= 1;
            lastErrY = errPos.y;
        }
        else
        {
            errPos.y = lastErrY;
        }
    }

    lastTime = gRobot.cvData.cvPeriod;
    lastAimObject = retifyType;

    gRobot.debugInfomation.lastRecPos.x = lastErrX;
    gRobot.debugInfomation.lastRecPos.y = lastErrY;
    gRobot.debugInfomation.xRecCnt = xRecCnt;
    gRobot.debugInfomation.yRecCnt = yRecCnt;
    gRobot.debugInfomation.grossRecErr.x = grossRecErr.x;
    gRobot.debugInfomation.grossRecErr.y = grossRecErr.y;
    gRobot.debugInfomation.aimObjectActPos.x = aimObjectActPos.x;
    gRobot.debugInfomation.aimObjectActPos.y = aimObjectActPos.y;
    gRobot.debugInfomation.firstColor = firstColor;
    gRobot.debugInfomation.calX =  calPosX[0];       
    gRobot.debugInfomation.calY =  calPosY[0];       
    gRobot.debugInfomation.calPoseAngle = calPoseAngle[0];  
    gRobot.debugInfomation.calPitchAngle = calPitchAngle[0];
    gRobot.debugInfomation.blackFirst = blackFirst;
    
    return errPos;
}

/**
  * @brief	用视觉矫正坐标
  * @name	CvRectifyPosPerTime
  * @note	None
  * @retval	None
  */
 void CvRectifyPosPerTime(void)
 {
    // 用视觉坐标矫正,需根据跑的阶段区分矫正的目标物
    if(GetY() < 4000.f && (gRobot.walkStatus == goForBlueTakeBall || gRobot.walkStatus == goForRedTakeBall))
    {
        gRobot.cvData.rectifyErr = CvRectifyPos(gRobot.cvData.ballBucketDis, gRobot.cvData.ballBucketAngle, BALL_TYPE);
    }
    else if(GetY() > 1200.f && (gRobot.walkStatus == goForBluePut1st || gRobot.walkStatus == goForRedPut1st))
    {
        gRobot.cvData.rectifyErr = CvRectifyPos(gRobot.cvData.ballBucketDis, gRobot.cvData.ballBucketAngle, BUCKET_TYPE);
    }
 }

//单字节递推中位值平均滤波

//功能:1.将新采样值压人队列

// 2.将队列中数据减去最大值和最小值,然后求平均值(小数四舍五人)

//入口: NEW_DATA  =    新采样值

//                QUEUE  =   队列

//                          n   =    队列长度

//出口:=滤波结果(平均值)

/**
  * @brief	V3速度计算加速度
  * @name	V3AccFilter
  * @note	None
  * @retval	None
  */ 
float V3AccFilter(float NEW_DATA)
{
    static float QUEUE[30] ={0.f};
    static float velLast;
    uint8_t n = 20;
    float max;
    float min;
    float sum;
    uint8_t i;

    static float sumLast;

    QUEUE[0]=NEW_DATA - velLast;       //新采样值入队列

	gRobot.expectCtrlT[2] = QUEUE[0];

    max=QUEUE[0];

    min=QUEUE[0];

    sum= QUEUE[0];

    for(i =n-1; i!=0;i--)

    {

        if (QUEUE[i]>max )           //比较并更新最大值

        max = QUEUE[i];

        else if (QUEUE[i] <min )    //比较并更新最小值

        min=QUEUE[ i];

        sum = sum + QUEUE[i];     //追加到和值

        QUEUE[i]=QUEUE[i-1];      //队列更新

    }

    i=n-2;

    sum =sum-max-min + i/2;

    sum =sum/i;            //平均值=(和值-最大值-最小值+n/2)/(队列长度-2)


	gRobot.expectCtrlT[0] = sum;
	gRobot.expectCtrlT[1] = NEW_DATA - velLast;
	gRobot.expectCtrlT[3] = QUEUE[0];
	gRobot.expectCtrlT[4] = QUEUE[8];
	gRobot.expectCtrlT[5] = QUEUE[9];
	gRobot.expectCtrlT[6] = NEW_DATA;
	gRobot.expectCtrlT[7] = sum *0.7f + 0.3f * sumLast;

                                //说明:+(n-2)/2的目的是为了四舍五人
    velLast = NEW_DATA;
    sumLast = sum;
    
    return (sum);

}

/**
  * @brief	根据辊子编码器调整车的姿态角
  * @name	AngleAdjustByStickPos
  * @note	None
  * @retval	返回要调整的姿态角
  */ 
float AngleAdjustByStickPos(float lStickPos, float rStickPos)
{
    static float lastLStick = 0.f, lastRStick = 0.f;
    float lPerVaria = 0.f, rPerVaria = 0.f;//每周期辊子编码器位置的变化量
    float angleAdjust = 0.f;
    //计算每个周期左、右编码器的位置变化量
    lPerVaria = gRobot.stickLPos - lastLStick;
    rPerVaria = gRobot.stickRPos - lastRStick;
    //编码器最大值为3fff，16656
    if(fabs(lPerVaria) > 10000)
    {
        lPerVaria = lPerVaria - lPerVaria/fabs(lPerVaria)*16656; 
    }
    if(fabs(rPerVaria) > 10000)
    {
        rPerVaria = rPerVaria - rPerVaria/fabs(rPerVaria)*16656; 
    }

    //计算要调整的角度
    if(fabs(lPerVaria - rPerVaria) > 20.f)//判断阈值
    {
        // angleAdjust = (rPerVaria - lPerVaria)*0.2f;//左辊子的编码器变化量小于右边则说明姿态角应该顺时针转，反之则逆
        angleAdjust = (rPerVaria - lPerVaria)*2.2f;
    }

    if (fabs(lPerVaria) < 10.f && fabs(rPerVaria) < 10.f)
    {
        angleAdjust = 0.f;
    }

    //限幅
    // if(fabs(angleAdjust) > 5.f)
    if(fabs(angleAdjust) > 80.f)
    {
        angleAdjust = angleAdjust/fabs(angleAdjust)*80.f;
    }

    lastLStick = gRobot.stickLPos;
    lastRStick = gRobot.stickRPos;

    gRobot.debugInfomation.lPerStick = lPerVaria;
    gRobot.debugInfomation.rPerStick = rPerVaria;
    return angleAdjust;
}
