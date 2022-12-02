#ifndef _PPS_H
#define _PPS_H

#include "stdint.h"
#include "process_comm.h"

#define GET_DATA_LEN (24)

typedef union 
{
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transPPSData_t;

typedef struct
{
	float x;
	float y;
	float angle;
	float speedX;
	float speedY; 
	float WZ;
	float correctX;
	float correctY;
	float correctAngle;
	float pitchAngle;
	float pitchSpeed;
}pps_t;

typedef struct
{
	float xVel;
	float yVel;
}velVector_t;

static pps_t ppsPos;
static pps_t realPos;
//等待mcu初始化
void Talk2Pps(void);

void WaitPpsPrepare(void);

void PpsDataRecognize(void);

void SetX(float setValue);

void SetY(float setValue);

void SetAngle(float setValue);

void SetCorrectX(float setValue);

void SetCorrectY(float setValue);

void SetCorrectAngle(float setValue);

void SetPitchAngle(float setValue);

void SetPitchSpeed(float setValue);



float GetCorrectX(void);

float GetCorrectY(void);

float GetCorrectAngle(void);

void SetSpeedX(float setValue);

void SetSpeedY(float setValue);

void SetWZ(float setValue);

float GetX(void);

float GetY(void);

float GetAngle(void);

float GetSpeedX(void);

float GetSpeedY(void);

float GetWZ(void);

float GetPitchAngle(void);

float GetPitchSpeed(void);

//返回减去绕机器人中心旋转角速度在定位系统位置产生的线速度后的速度
velVector_t GetSpeedWithoutOmega(void);

extern char writePpsData[MC_LIDAR_BUF_LENGTH];
#endif
