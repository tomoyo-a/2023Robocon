#ifndef _POS_H
#define _POS_H

#include "map.h"
//opencv头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

//角度制转换为弧度制
#define ANGTORAD(ang) (ang / 180.f * CV_PI)
//弧度制转换为角度制
#define RADTOANG(rad) (rad / CV_PI * 180.f)

//斜率为能容忍的最大偏差
#define ANGERRMAX (2.f)
//定义场地的长度X
#define YARDWIDETH 5925
//定义场地的长度Y
#define YARDLENTH 11900
#ifdef DRMODE
//定义定位系统到雷达的距离
#define PPSTOLIDAR (271.89)
#define PPSANGLE (150)
//定义初始位姿
#define INITXPOS (5330)
#define INITYPOS (11542)
//初始位置解算点
//初始化起始点Y
#define INITONESTART (260)
//初始化结束点
#define INITONEEND (460)
//初始化起始点X
#define INITTWOSTART (640)
//初始化结束点
#define INITTWOEND (820)
#endif

#ifdef TRMODE
//定义定位系统到雷达的距离
#define PPSTOLIDAR (271.89)
#define PPSANGLE (270)
//定义初始位姿
#define INITXPOS (580)
#define INITYPOS (485)
#define INITAPOS (-90)
// #define PPSANGLE (150)
// //定义初始位姿
// #define INITXPOS (800)
// #define INITYPOS (4000)
//初始位置解算点
//初始化起始点Y
#define INITONESTART (480)
//初始化结束点
#define INITONEEND (620)
//初始化起始点X
#define INITTWOSTART (700)
//初始化结束点
#define INITTWOEND (840)
#endif
//初始化时最小角度
#define INITANGLE (5)
//x最大单次矫正值
#define CORRECTXMAX (7)
//errorMAX
//#define ERRORMAX (1000)
//y最大单次矫正值
#define CORRECTYMAX (10)

//场地匹配
void MatchMap(void);

//返回匹配直线
int MatchLine(LineParam_t calculateLine);

//判断点是否在多边形内部
bool PointInPolyTest(cv::Point point, std::vector<LineParam_t> lines);

/*通过直线计算世界坐标
 *@param lineForCorrect 用于计算坐标的所有直线
 *@return 返回世界坐标系下 x,y,theta 的累计矫正量
 */
Pos_t CorrectPos(Pos_t rbotPos,vector<LineParam_t>& lineForCorrect);

//角度矫正
bool CorrectAngle(vector<LineParam_t> lines,float maxLineAngErr);

//除了初始位置时，用雷达矫正坐标的情况
bool SetPosFlag();

// 补偿车到墙计算出的值
void CompenDistance(int &distance);

void CompenStartDis(int &distance);
#endif
