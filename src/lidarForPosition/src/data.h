#ifndef _DATA_H
#define _DATA_H

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <atomic>
//opencv头文
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

#define POINTNUM 1440
#define POINTMINNUM (POINTNUM / 5)
#define ANGERR 15
#define ANGMAX 180
#define DISERR 0.1f
using namespace std;
using namespace cv;

typedef struct
{
    float x;
    float y;
    float theta;
    float speedX;
    float speedY;
    float angleW;
} PPSData_t;
typedef class _Pos_t_
{
public:
    _Pos_t_(){};
    ~_Pos_t_(){};

    inline _Pos_t_(const float _x, const float _y, const float _theta)
        : x(_x), y(_y), theta(_theta){};

    inline _Pos_t_(const float _x, const float _y)
        : x(_x), y(_y){};

    inline _Pos_t_(const cv::Point2f &_p, float _theta)
        : x(_p.x), y(_p.y), theta(_theta){};

    inline _Pos_t_(const cv::Point2f &_p)
        : x(_p.x), y(_p.y){};

    inline _Pos_t_ operator+(const _Pos_t_ &a)
    {
        _Pos_t_ ret;
        ret.x = this->x + a.x;
        ret.y = this->y + a.y;
        ret.theta = this->theta + a.theta;
        return ret;
    }
    inline _Pos_t_ operator-(const _Pos_t_ &a)
    {
        _Pos_t_ ret;
        ret.x = this->x - a.x;
        ret.y = this->y - a.y;
        ret.theta = this->theta - a.theta;
        return ret;
    }

    inline void operator=(const _Pos_t_ &a)
    {
        x = a.x;
        y = a.y;
        theta = a.theta;
    }

    inline _Pos_t_ &operator+=(const _Pos_t_ &a)
    {
        x += a.x;
        y += a.y;
        theta += a.theta;
        return *this;
    }

    inline _Pos_t_ &operator-=(const _Pos_t_ &b)
    {
        x -= b.x;
        y -= b.y;
        theta -= b.theta;
        return *this;
    }

    friend ostream &operator<<(ostream &out, _Pos_t_ &a)
    {
        out << a.x << ' ' << a.y << ' ' << a.theta;
        return out;
    }

    float x = 0;
    float y = 0;
    //角度
    float theta = 0;
} Pos_t;

typedef class _LineParam_t_
{
public:
    _LineParam_t_(){};
    ~_LineParam_t_(){};

    inline _LineParam_t_(const int _A, const int _B, const int _C)
        : A(_A), B(_B), C(_C){};

    inline _LineParam_t_(const int _ZERO)
        : A(0){};

    inline _LineParam_t_ operator+(const _LineParam_t_ &a)
    {
        _LineParam_t_ ret;
        ret.A = this->A / 2 + a.A / 2;
        ret.B = this->B / 2 + a.B / 2;
        ret.C = this->C / 2 + a.C / 2;
        ret.angle = this->angle / 2 + a.angle / 2;
        ret.startP = this->startP; //前一条直线起点为+后的起点，后一条直线终点为+后的终点
        ret.endP = a.endP;
        ret.startPoint = this->startPoint;
        ret.endPoint = a.endPoint;
        ret.pNum = this->pNum + a.pNum;
        return ret;
    }

    void operator=(const _LineParam_t_ &mid)
    {
        A = mid.A;
        B = mid.B;
        C = mid.C;
        slope = mid.slope;
        intercept = mid.intercept;
        pNum = mid.pNum;
        angle = mid.angle;
        radar2lineDistance = mid.radar2lineDistance;
        startPoint = mid.startPoint;
        endPoint = mid.endPoint;
        startP = mid.startP;
        endP = mid.endP;
        visible = mid.visible;
        wallNum = mid.wallNum;
    }
    friend ostream &operator<<(ostream &out, _LineParam_t_ &a)
    {
        out << a.startP << ' ' << a.endP << ' ' << a.pNum << ' ' << a.wallNum << ' '
            << a.A << ' ' << a.B << ' ' << a.C << ' ' << a.angle << ' ' << a.rbot2Line;
        return out;
    }

    inline _LineParam_t_ &operator+=(const _LineParam_t_ &a)
    {
        A += fabs(a.A - A) / 2;
        B += fabs(a.B - B) / 2;
        C += fabs(a.C - C) / 2;
        pNum += a.pNum;
        startP = this->startP; //前一条直线起点为+=后的起点，后一条直线终点为+=后的终点
        endP = a.endP;
        startPoint = this->startPoint;
        endPoint = a.endPoint;
        return *this;
    }

    float A = 0.f;
    float B = 0.f;
    float C = 0.f;
    float slope = 0.f;
    float intercept = 0.f;
    //角度均用角度制 范围为(-90,90]
    float angle = 0.f;
    float pNum = 0.f;
    float rbot2LineX = 0.f;
    float rbot2Line = 0;
    float radar2lineDistance;
    cv::Point startPoint;
    cv::Point endPoint;
    int startP = 0;
    int endP = 0;
    int wallNum = -1;
    float rbot2LineY = 0.f;
    bool visible;
    vector<cv::Point> linePoint;
    cv::Point calXYErr;
} LineParam_t;

typedef enum
{
    CORRECTANGLE = 0,
    CORRECTPOS,
    CORRECTNOMAL
} _mode_t;

//清空数据
void DataClear(void);

//数据处理,找到所有的角点并保存
void DataProcess(void);

//找到角点
int FindCornerPos(int start, int end);

//保存该次数据中的角点
void SaveCornerPos(int start, int end);

//计算直线参数
void GetLine(void);

//最小二乘法拟合直线
LineParam_t LineFitLeastSquares(vector<cv::Point> radarPoint, int start, int end);

//对容器内的角点进行排序
void Sort(void);

//对拟合的直线进行排序，长的排在前面
void SortLine();

//计算点到直线距离
float P2LineDistance(LineParam_t line, cv::Point point);

//对雷达的数据进行动态补偿
void DataCompensation(vector<Pos_t> posData, vector<Pos_t> &compenData);

//对已知数据进行坐标变换
cv::Point DataTransform(cv::Point oriPoint, int xTransform, int yTransform, float radTransform);

//判断点是否在场地内
int WhetherPointInSite(Point pt);

//限制角度范围
float LimitAng(float &ang);

//合并直线
void MergeLine(vector<LineParam_t> &allCalculateLine);

//判断两条直线是否为一条直线
bool JudgeSameLine(LineParam_t line1, LineParam_t line2);

// 按顺序传入点来计算两点的直线(p1为起点，p2为终点) 以逆时针方向传入点
LineParam_t TwoPFittLine(Point p1, Pos_t p2);

// 判断直线是否相交，如果相交，返回值为交点，不相交，返回值为原点
Pos_t TwoLineIfIntersect(LineParam_t line1, LineParam_t line2);

// 按照直线是否被遮挡的方法来分割图像上的点，并且转换成雷达扫描到的点的标号，记录改方向下看到的直线标号
void DivisionImage(vector<int> &keyPoint, vector<int> &wallNum);

// 筛选雷达扫描到的点
void ScreenRadarPoint();

/*求取vector内数据均值
 *@param data: 传入数据
 *@return 容器内数据均值
 */
template <typename T>
T AvgVector(vector<T> data);

/*求取vector内数据均方差
 *@param data: 传入数据
 *@return 容器内数据均方差
 */
template <typename T>
float StdVector(vector<T> data);

/*求取vector内数据和
 *@param data: 传入数据
 *@return 容器内数据之和
 */
template <typename T>
T SumVector(vector<T> data);

class Robot_t
{
public:
    int drStatus = 1;
    float ppsInitAngle = 0;
    float lidarRotation = ppsInitAngle + 90;
    Pos_t ppsCorrect;
    Pos_t singleCorrect;
    Pos_t ppsErr;
    atomic_int ppsCorrect_fx;
    atomic_int ppsCorrect_fy;
    atomic_bool serialFlag;
    vector<Pos_t> recentPosData;
    vector<long> lidarData;
    vector<double> radarTheta;
    vector<int> radarDistance;
    vector<cv::Point> radarPoint;
    vector<cv::Point> mapLidarPoint;
    Point lidarCalPoint;
    deque<Pos_t> posData;
    Pos_t rbotPos;
    Pos_t ppsRawPos;
    _mode_t mode = CORRECTANGLE; //默认先矫正角度
    //车的速度（包含X Y速度和角速度）
    Pos_t rbotVel;
    chrono::steady_clock::time_point timeStart;
    chrono::steady_clock::time_point timeEnd;
    void OutputInformation(ofstream &outData);
};

#endif
