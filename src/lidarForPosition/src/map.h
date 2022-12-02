#ifndef _MAP_H
#define _MAP_H

#include <iostream>
#include "string.h"
#include <thread>
#include "math.h"
#include "stdio.h"
#include "data.h"
//opencv头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

//定义是否是DR模式
// #define DRMODE
#define TRMODE
//#define UNLIDAR
//测试雷达是否被遮挡
//#define TEST_LIDAR
void InitMap(void);
void ShowMap(cv::Mat mapImage, std::vector<LineParam_t> wallLine);

#ifndef TEST_LIDAR
cv::Point P2Img(cv::Point p, int originX = 100, int originY = 720, int scale = 20);
#else
cv::Point P2Img(cv::Point p, int originX = 100, int originY = 720, int scale = 20);
#endif


#endif