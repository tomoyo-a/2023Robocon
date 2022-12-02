#ifndef __BALANCE_H
#define __BALANCE_H

#include <stdint.h>
#include "MotionCard.h"

//目标矫正物
#define BAFFLE_TYPE    0 //中间的墙
#define LEFT_WALL_TYPE 1 //侧边墙
#define BALL_TYPE      2 //球
#define BUCKET_TYPE    3 //桶

//每个周期最大调整量
#define MAX_ADJUST_PER_TIME 1.f

//识别到第一个是黑球的周期数
#define BLACK_BALL_PER_TIME 2

//视觉连续识别到该周期 开始矫正
#define CV_PER_TIME 3

/**
  * @brief	用视觉识别到的角度和距离计算矫正量,会不会出现矫正量过大的情况？？
  * @name	CvRectifyPos
  * @note	None
  * @param	relativeDis 视觉的相对距离
  * @param	relativeAngle 视觉的偏角
  * @param  retifyType 视觉识别的东西,分别为中间的墙、球、桶，现在三种并无区别
  * @retval	None
  */
Point_t CvRectifyPos(float relativeDis,float relativeAngle, uint8_t retifyType);

/**
  * @brief	用视觉矫正坐标
  * @name	CvRectifyPosPerTime
  * @note	None
  * @retval	None
  */
 void CvRectifyPosPerTime(void);

 /**
  * @brief	V3速度计算加速度
  * @name	V3AccFilter
  * @note	None
  * @retval	None
  */
float V3AccFilter(float NEW_DATA);

/**
  * @brief	根据辊子编码器调整车的姿态角
  * @name	AngleAdjustByStickPos
  * @note	None
  * @retval	None
  */ 
float AngleAdjustByStickPos(float lStickPos, float rStickPos);

#endif