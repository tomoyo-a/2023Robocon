#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "calculate.h"
#include "moveBase.h"

//红场
// #define RED_COURT_DEF

//CV communication
#define CV_COM
//LIDAR communication
// #define LIDAR_COM
//SPI communication
#define SPI_COM

#define FIRST_PATH_PERCENT 0.3f
#define SECOND_PATH_PERCENT 0.2f
#define THIRD_PATH_PERCENT 0.3f
#define PUT_SED_PATH_PERCENT 0.3f
#define PUT_THIRD_PATH_PERCENT 0.3f
#define PUT_FOURTH_PATH_PERCENT 0.3f
#define PUT_FIFTH_PATH_PERCENT 0.3f
#define BLUE_PUTTING_PATH_PERCENT 0.2f//边走边放需要速度慢一点

//红蓝场坐标x的切换,红场是负的，蓝场是正的
#define COUTR_COLOR_X ColorCoutrX(gRobot.cvData.filed)

//模式选择
#define OPERATING_MODE				NORMAL_MODE

//红场,暂且定为与蓝场的x相反
#define RED_COURT (1)
//蓝场
#define BLUE_COURT (0)
/*机器人模式*/
//复位模式
#define RESET_MODE (0)
//走形模式
#define WALK_MODE (1)
/*走形模式*/
#define ATTACK_MODE (1)//进攻
//防御模式
#define DEFENCE_MODE (2)//防御
//自检模式
#define SELF_CHECK_MODE (4)
//调参模式
#define DEBUG_PARA_MODE (5)
//重试模式
#define RETRY_MODE (6)
//测试加速度
#define ACC_TEST				(2)
#define NO_PLANNING (0)   
#define IS_PLANNING (1)

//球序号
#define PINK  1
#define GREEN 2
#define BLUE  3
#define BLACK 4
#define WHITE 5
#define NONE 6

//桶序号
#define FIRST_BUCKET 1
#define SECOND_BUCKET 2
#define THIRD_BUCKET 3
#define FOURTH_BUCKET 4
#define FIFTH_BUCKET 5

//调试数据结构体
typedef struct
{
	//PathFollowing轨迹跟随变量
	float VIEW_L;
	float robotlenVP;
	float robotlenVT;
	PointU_t virtualPos;
	PointU_t virtualTarget;
	float disRealPos2VirPos;
	float disRealPos2VirTarget;
	float disAdd;
	float posAngleVP;
	float posAngleVT;
	float omega;
	float originVel;
	float originVelDir;
	float fixedVel;
	robotVel_t adjustVel;
	float sumVel;
	float sumVelDir;
	float distance;

	//预测点信息
	float predictLen;
	PointU_t predictPos;
	float predictVel;
	float predictVel_L;
	float predict_U;

	float predictLen1;
	PointU_t predictPos1;
	float predictVel1;
	
	//VelControl速度环变量
	float velXErr;
	float velYErr;
	float velXErrL;
	float velYErrL;
	float outputVel;
	float outputDirection;
	float outputOmega;
	float angleVelOUtput;

	// float target
	float angleAim;
	float angleKd;
	float angleKp;
	float errSum;

	//
	float adjustvellast;
	float adjustDistance;

	float search[10];

	//视觉坐标矫正debug
	Point_t cvWorldPos;
	float cvDis;
	float cvAngle;
	float rollErrPosX;
	Point_t lastRecPos;
	int xRecCnt;
	int yRecCnt;
	Point_t grossRecErr;
	Point_t oriGrossRecErr;
	Point_t aimObjectActPos;
	uint8_t firstColor;
	float takingAngle;
	float calX;
	float calY;
	float calPoseAngle;
	float calPitchAngle;
	uint8_t blackFirst;

	//辊子调整姿态角的发数
	float lPerStick;
	float rPerStick;
}debugInfo_t;

//调试数据结构体
typedef struct
{
	//
	uint8_t NowposIndex;
	uint8_t totalNum;
	uint8_t judgeHalfPoint;
}planData_t;
//走行状态变量枚举类型变量
typedef enum
{
	//蓝场
	waitForBlueTakeBall,	//0
	goForBlueTakeBall,//1
	waitForBlueTakingBall,//2
	goForBlueTakingBall,//3
	waitForBluePut1st,//4
	goForBluePut1st,//5
	waitForBluePut2nd,//6
	goForBluePut2nd,//7
	waitForBluePut3rd,//8
	goForBluePut3rd,//9
	waitForBluePut4th,//10
	goForBluePut4th,//11
	waitForBluePut5th,//12
	goForBluePut5th,//13
	waitForBluePutting,//14
	goForBluePutting,//15
	//红场
	waitForRedTakeBall,	//16
	goForRedTakeBall,//17
	waitForRedTakingBall,//18
	goForRedTakingBall,//19
	waitForRedPut1st,//20
	goForRedPut1st,//21
	waitForRedPut2nd,//22
	goForRedPut2nd,//23
	waitForRedPut3rd,//24
	goForRedPut3rd,//25
	waitForRedPut4th,//26
	goForRedPut4th,//27
	waitForRedPut5th,//28
	goForRedPut5th,//29

	testPara,
	stop
}walkStatus_t;

//与cv通信的结构体
typedef struct
{
	uint8_t cvPeriod;//视觉周期
	uint8_t filed;//红蓝场变量 
	uint8_t ballOrBucket;//取球球或放球
	uint8_t baffleOrLeft;//过道或侧墙
	uint8_t colorFlag;	//颜色识别状态标志位
	uint8_t locateFlag;	//矫正识别球或桶标志位
	uint8_t lBaffleFlag;//侧墙或者过道的识别状态
	uint8_t colorMsg[5];//球或者桶的颜色信息
	float ballBucketAngle;//球或桶的角度
	float ballBucketDis;//球或桶的距离
	float baffleAngle;//过道中心角度
	float lBaffleDis;//过道中心或侧墙距离
	float wallYawAngle;//放球区墙的偏角，左正右负
	Point_t rectifyErr;
}cvData_t;

//全局变量结构体
typedef struct
{
	//轮子状态
	wheelState_t wheelState;
	//调试数据
	debugInfo_t debugInfomation;
	//走行状态
	walkStatus_t walkStatus;
	//
	planData_t  planData;
	//与CV通信结构体
	cvData_t cvData;
	//mcu通信心跳包
	int mcuHeart;
	//pps心跳包
	int ppsHeart;
	int ppsSameHeart;
	//红蓝场
	uint8_t courdID;
	//轨迹规划ing 标志位
	uint8_t pathPlanFlag;
	//机器人模式
	uint8_t robotMode;
	//走形模式
	uint8_t walkMode;
	//防守的桶号
	uint8_t defencePotID;
	//进攻桶号
	uint8_t attackPotID;
	//开场红蓝场信息
	uint8_t colorFlag;
	//射箭命令
	uint8_t archeryStart;
	//射完箭命令
	uint8_t archeryDone;
	//挥箭命令
	uint8_t waveArchery;
	//MCU's angle
	int cvMcuDir;
	//视觉传过来的偏离角度
	float cvDir;
	//视觉传过来的距离
    float cvDis;
	//转盘角度
	float turnAngle;
	//CV state
	uint8_t CVstate;

	uint8_t fetchReady;
	//已射箭的个数
	uint8_t shootArrowCnt; 
	//重试标志位
	uint8_t TRRetryFlag;
	//重试取箭标志
	uint8_t writeFlag;
	//重试标志位
	uint8_t DRRetryFlag;
	//LiDar返回信号，2是他收到了；3是他矫正完了
	uint8_t DRRetryReturn;
	//取箭次数
	uint8_t takeCnt;
	//在出发区重试取箭
	uint8_t tryTakeFlag;
	//给视觉的桶号标志位(射箭时再赋值)
	uint8_t cvAttackPotID;

	//新增自动射箭数据
	float autoShootVel;
	float voltageTemp;
	float autoShootKp;
	float autoShootKi;
	float autoPitchAngle;

	//omg
	float robotVel;
	float velDir;
	float omg;
	//preDictOmg
	float preDictVel;
	float preDictVelDirec;
	float preDictPoseAngle;
	float expectCtrlT[10];
	float everOmg;
	float everVel;
	float everVelDirec;

	//locus over flag
	uint8_t velControlStop;
	uint8_t ppsCnt;
	uint8_t mcuCnt;

	uint8_t momentumTime;

	uint8_t predictChange;

	float V3AccCaculate;

	//辊子编码器读数，用来控制姿态角
	float stickLPos;
	float stickRPos;

	//桶的颜色顺序
	uint8_t bucketColor[5];

	//放球标志位
	uint8_t putBallFlag;
	uint8_t putBallOverFlag;

	//up device have gotten ball num
	uint8_t ballNum;
	uint8_t gamestart;

	//视觉连续识别到坐标
	uint8_t cvPeriodFlag;
}gRobot_t;

extern gRobot_t gRobot;
int GetTimeCounter(void);

void CountTime(void);

void SetCountTimeFlag(void);

void Walk(void);

uint8_t JudgeSpeedLessEqual(float speedCompared);

void VelControl(robotVel_t actVel);

//5号桶切换到1号桶
void DefencePot5toPot1(void);

//5号桶切换到2号桶
void DefencePot5toPot2(void);

//1号桶切换到2号桶
void DefencePot1toPot2(void);

//2号桶切换到1号桶
void DefencePot2toPot1(void);

//1号桶切换到5号桶
void DefencePot1toPot5(void);

//测试加速度程序
void TestParaMotor(void);

//射箭区雷达矫正（为取箭做矫正）
void LidarCorrect(void);

int FetchDone(void);

/**
* @name   LimitIncludeAngle
* @brief  限制夹角在+1~ -1之间,并计算出结果
* @param  angleOne 角度one
* @param  angleTwo 角度two
* @retval 返回正确夹角
*/
float LimitIncludeAngle(float angleOne,float angleTwo);

/**
  * @brief	jiao速度输出六个周期的平均值
  * @name	Get_AverOmg
  * @note	None
  * @param	omg
  * @retval	六个周期的平均值
  */
float Get_AverOmg(float omg);

/**
  * @brief	速度输出六个周期的平均值
  * @name	Get_AverVel
  * @note	None
  * @param	vel
  * @retval	六个周期的平均值
  */
float Get_AverVel(float vel);


/**
  * @brief	速度方向输出六个周期的平均值
  * @name	Get_AverVelDirec
  * @note	None
  * @param	vel
  * @retval	六个周期的平均值
  */
float Get_AverVelDirec(float velDirec);

/**
  * @brief	用车的垂直方向的加速度计算平衡角度
  * @name	AccBlanceAngle
  * @note	None
  * @param	gyroAcc
  * @retval	None
  */void AccBlanceAngle(void);

  /**
  * @brief	根据红蓝裳选择x的正负，蓝场x是正的，红场x是负的
  * @name	ColorCoutrX
  * @note	None
  * @retval	X轴的符号
  */
float ColorCoutrX(uint8_t courtColor);

/**
  * @brief	judgeTakingBallVel
  * @name	judgeTakingBallVel
  * @note	None
  * @retval	veljudge
  */
uint8_t judgeTakingBallVel(float vel);


#endif

