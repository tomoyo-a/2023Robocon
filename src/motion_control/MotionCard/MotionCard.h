#ifndef _MOTIONCARD_H
#define _MOTIONCARD_H

//注意 示教模式下所存点存储在 0x08040000 地址中  stm32f407单片机

//点的结构体 单位mm
typedef struct
{
	float x;
	float y;
}Point_t;

//点斜式结构体 ，斜率用角度制的角度代替
typedef struct
{
	Point_t point;
	//角度制
	float   direction;
	//速度
	float vel;
}Pose_t;

typedef struct
{
	float v1;
	float v2;
	float v3;
	float v4;
}TriWheelVel_t;


//ACCMAX摩擦力和电机能提供的最大加速度 mm/s^2
extern float GetAccMax(void);
//VELLMAX机器人能达到的最大速度 mm/s  该值是通过比较摩擦力提供的最大加速度与电机能提供给机器人的最大加速度，取最小值。
extern float GetVelMax(void);

//开辟内存，使用前必须首先调用  num 最大存取点个数，一个点占28字节  ;返回1代表开辟成功
int BufferZizeInit(int num);

//路径跟随函数 percent: 范围为0~1.1代表规划的最大速度;返回 -1代表传入参数超出范围；返回1代表成功运行
int PathFollowing(float percent);


//示教采点函数，采点时，需要一直调用;返回1代表成功采集
int PoseSampling(void);

//读取flash里的示教点信息，并添加到ringBuffer里;  返回 1 代表成功读取
int ReadFlashPointInformation(void);

//采点完成时需要调用，将所采点信息填入flash里;返回 1 代表成功读取
int PoseSamplingDone(void);

//清空ringBuffer,一旦清空，底盘将会停车抱死
void ClearRingBuffer(void);

//清空内存池后，填入新的关键点，进行路径规划; 返回1代表函数成功执行
int InputPoints2RingBuffer(Pose_t *points,int num);


//该函数用于记录机器人行走的路径，必须全程调用！！！
void CaculatePath(void);

////本人调试所用
//int GetCount(void);

//计算预测路径时间
float GetPredictTime(void);

//float GetRingBufferPointPoseAngle(int num);
//float GetRingBufferPointAngle(int num);
//float GetRingBufferPointVell(int num);
//float GetRingBufferPointLen(int num);
//float GetRingBufferAverCurvature(int num);
//float GetLength(void);
//Point_t GetRingBufferPoint(int num);
#endif



