#ifndef __VISION_H
#define __VISION_H

#include "stdint.h"
typedef union 
{ 
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transCVData_t;

#define SEND_COLOR_DATA (5)
#define SEND_POT_DATA (22)

#define SEND_CV_DATA (33)

// //color序号
// #define PINK  0
// #define GREEN 1
// #define BLUE  2
// #define BLACK 3
// #define WHITE 4
// #define NONE 5

//task
#define BALL  	0
#define BUCKET  1
#define BAFFLE  0
#define LEFT  	1

//field
#define BLUE_FIELD  0
#define RED_FIELD 	1

//等待cv初始化
void WaitCvPrepare(void);

//定周期运行
void Talk2Cv(void);

//接受视觉信息
void CvDataRecognize(void);

//接收球或桶的角度
void SetBallBucketAngle(float setValue);

//接收球或桶的距离
void SetBallBucketDis(float setValue);

//接收过道中心角度
void SetBaffleAngle(float setValue);

//接收过道中心或侧墙距离
void SetlBaffleDis(float setValue);

//接收放球区墙的偏角
void SetWallYawAngle(float setValue);

//将红蓝场、桶号、x、y、转盘角度传给视觉
void SendCVData(void);


#endif 
