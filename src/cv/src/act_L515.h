#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_L515_H_
#define ACT_L515_H_

#include <iostream>
#include <fstream>
#include <mutex>
#include <thread>
#include <sstream>
#include <math.h>
#include <time.h>
#include <numeric>
#include <cmath>
#include <atomic>

#include <Eigen/Dense>//使用cv2eigen,先包含eigen相关库，再包含opencv2/core/eigen.hpp

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include<opencv2/core/eigen.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rs_advanced_mode.h>

extern "C"
{
	#include "process_comm.h"
}

#define DEBUG        	//测试员模式
#define RECORD_VIDEO		//过程录像
#define TXT 			//运行数据记录
// #define TIME			//程序计时
#define IFCAMERA		//已连接相机
#define COMMUN			//进程通信

#define IMAGE_WIDTH 	640
#define IMAGE_HEIGHT 	480
#define DEPTH_WIDTH 	640
#define DEPTH_HEIGHT 	480

#define ROBOTCENTER_X 	0.00
#define ROBOTCENTER_Y 	0.199f
#define ROBOTCENTER_Z 	0.3045f

//识别球墙参数
#define BALLWALL_MAX_H_L 20
#define BALLWALL_MIN_H_L 0
#define BALLWALL_MAX_H_R 180
#define BALLWALL_MIN_H_R 180
#define BALLWALL_MAX_S   255
#define BALLWALL_MIN_S   130
#define BALLWALL_MAX_V   255
#define BALLWALL_MIN_V   30

//识别红墙参数
#define REDWALL_MAX_H_L 20
#define REDWALL_MIN_H_L 0
#define REDWALL_MAX_H_R 180
#define REDWALL_MIN_H_R 177
#define REDWALL_MAX_S   255
#define REDWALL_MIN_S   50
#define REDWALL_MAX_V   255
#define REDWALL_MIN_V   30

//识别蓝墙参数
#define BLUEWALL_MAX_H_L 129
#define BLUEWALL_MIN_H_L 80
#define BLUEWALL_MAX_H_R 180
#define BLUEWALL_MIN_H_R 180
#define BLUEWALL_MAX_S 	 255
#define BLUEWALL_MIN_S 	 100
#define BLUEWALL_MAX_V 	 255
#define BLUEWALL_MIN_V 	 0

// //识别球参数
// #define BLACK_GREEN_BLUE_THRESOLD 40
// #define PINK_THRESOLD 160
// #define WHITE_THRESOLD 160

//颜色判别部分
#define PINK_MAX_H 180
#define PINK_MIN_H 141
#define PINK_MAX_S 255
#define PINK_MIN_S 50

#define GREEN_MAX_H 101
#define GREEN_MIN_H 36
#define GREEN_MAX_S 255
#define GREEN_MIN_S 68
#define GREEN_MAX_V 255
#define GREEN_MIN_V 60

#define BLUE_MAX_H 142
#define BLUE_MIN_H 91
#define BLUE_MAX_S 255
#define BLUE_MIN_S 90
#define BLUE_MAX_V 255
#define BLUE_MIN_V 50

#define BLACK_MAX_V 65
#define BLACK_MIN_V 0

#define WHITE_MAX_S 90
#define WHITE_MIN_S 0
#define WHITE_MAX_V 255
#define WHITE_MIN_V 70

//球序号
#define PINK  1
#define GREEN 2
#define BLUE  3
#define BLACK 4
#define WHITE 5
#define NONE 6

//task
#define BALL  	0
#define BUCKET  1
#define BAFFLE  0
#define LEFT  	1

//field
#define BLUE_FIELD  0
#define RED_FIELD 	1

using namespace std;
using namespace cv;
using namespace rs2;
using namespace Eigen;
using namespace rs400;

//-- ROI of an object
typedef struct
{
	double xMin;
	double xMax;

	double yMin;
	double yMax;

	double zMin;
	double zMax;

} ObjectROI;

struct float3
{
    float x, y, z;

    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return {x - t, y - t, z - t};
    }

	float3 operator-(float3 other)
	{
		return {x - other.x, y - other.y, z - other.z};
	}

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }

	void toDegree(void)
	{
		x = x / CV_PI * 180.f;
		y = y / CV_PI * 180.f;
		z = z / CV_PI * 180.f;
	}
};

struct TargetOutPut
{
	char color;
	Point2f center;
	Point3d real3D; 
};

union dataUnion
{
	char dataChar[4]={0};
	float data;
};

bool compContours(const vector<Point> &a, const vector<Point> &b);
bool compTargetLeftToRight(const TargetOutPut &a, const TargetOutPut &b);
bool compTargetRightToLeft(const TargetOutPut &a, const TargetOutPut &b);
void CopyData(char* origen, char* afterTreat, int size);

// typedef pcl::PointXYZRGB 			pointType;				//点云类型
// typedef pcl::PointCloud<pointType> 	pointCloud;				//点云
// typedef pointCloud::Ptr 				pPointCloud;			//点云指针

/******************IMU*******************/
inline void processGyro(rs2_vector gyro_data, double ts);
inline void processAccel(rs2_vector accel_data);
bool check_imu_is_supported(void);

class ActL515
{
public:
	ActL515();
	ActL515(const ActL515&) = delete;
	ActL515 operator=(const ActL515&) = delete;
	~ActL515();

	void Init(void);												//初始化
	inline void Update(rs2::frameset fs);									//更新图像
	inline void imagePreProcess(void);										//图像预处理
	void showImage(void);											//显示图像
	void recordData(void);
	void release(void);												//释放图像

	void getCameraExtrinsics(rs2::frameset alignProcess);

	bool SortColorNew(char field, char task);

	bool ImageDepthFilter(Mat &inImage, Rect roi, int lowDepth, int upDepth, char maxVal);		//深度滤波							
	TargetOutPut ColorJudge(Point center);														//颜色判断
	bool depthJudge(int x, int y);																//像素点深度判断函数

	void getMCmessage(void);                        //获得运动控制通信数据,发送和接收的顺序都是combine中的顺序
	char SendMessage(void);							//发送数据给运动控制

	// std::tuple<uint8_t, uint8_t, uint8_t> GetColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	// pPointCloud PointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);
	// -- For point cloud without color
	// pPointCloud PointsToPointCloud(const rs2::points& points);

	/**************OpenCv*************/
	cv::Mat 				srcImage;				//源图片，彩色图片source image
	cv::Mat         		depthImage;				//深度图
	cv::Mat         		depthHotMap;			//深度热力图
	cv::Mat 				HSVImage;				//HSV图
	cv::Mat 				rotateImage;			//旋转后图像
	cv::Mat 				rotateDepthImage;		//旋转后深度图像
	cv::Mat 				dstImage;				//旋转后深度图像

	time_t timeReal;
	tm* t;
	ofstream Fout;
	
public:
	uint16_t* data;
	// pPointCloud	cloudByRS2;					//realsense2传感器获得的点云
private:

private:
	rs2::context 			ctx;
	size_t 					device_count;	
	rs2_intrinsics 			color_intrin;			//颜色相机内参
	rs2_intrinsics 			depth_intrin;			//深度相机内参
	rs2_extrinsics 			depth2color_extrin;		//深度向颜色外参
	rs2_extrinsics 			color2depth_extrin;
	rs2::pointcloud  		rs2Cloud;				//深度图计算得到的点云 RS2指realsense2系列相机
	rs2::points      		rs2Points;				//点云格式的点
	rs2::align       		alignToColor;			//对齐图像
	rs2::pipeline    		pipe;					//数据传输管道
	rs2::config      		cfg;					//初始化
	rs2::frameset    		frameSet;				//帧设置
	rs2::frameset    		alignProcess;
	rs2::colorizer  		color_map;				
	rs2::temporal_filter 	tem_filter;				//时间过滤器
	rs2::spatial_filter 	spat_filter;			//

	Mat imageRotationMatix;							//彩色旋转矩阵
	Mat tempRotationMatix;							//旋转矩阵中转站

	Point2f imgCenter;

	//通讯相关
	char filed = 0;														//红蓝场变量 blue 0, red 1
	char ballOrBucket = 0;												//取球球或放球 ball 0, bucket 1 
	char baffleOrLeft = 0;												//过道或侧墙 baffle 0, left 1
	char colorFlag = 0;													//颜色识别状态标志位
	char locateFlag = 0;												//矫正识别标志位
	char lBaffleFlag = 0;							
	char colorMsg[5] = {NONE, NONE, NONE, NONE, NONE};									//存放颜色顺序，球从左到右，桶从右到左
	float ballBucketAngle = 0.f;												//球或桶的角度, 左正右负
	float ballBucketDis;												//球或桶的距离, 球或桶中心到车中心
	float baffleAngle;													//过道中心角度, 左正右负
	float lBaffleDis;													//过道中心或侧墙距离, 过道中心或侧墙横向距离（平行时）
	float wallYawAngle;													//机器人相对墙平面法向量的偏航角, 左正右负

	float robotRoll;													//机器人横滚角, 右正左负
	float robotYaw;														//机器人偏航角, 左正右负
	float robotPitch;													//机器人俯仰角
	float robotX;														//机器人X,原点蓝场为左下角，红场为右下角，Y正方向为正前方，X正方向为右方

	float lastTargetAngle = 0.f;
	
	bool timeFlag = 0;													//周期心跳包

	char cvData[MC_CV_BUF_LENGTH] = {0};
    char recCvData[MC_CV_BUF_LENGTH] = {0};        //通信数据储存数组

	//for Identify			
	float sideWallCameraX;												//相机侧墙在相机坐标系下的X,球是左侧，桶是右侧
	AngleAxisd rotateVector;											//相机3D坐标系旋转向量
	Matrix<double, 2, 3> imageInvRotateMatrix;							//图像逆旋转矩阵
	Matrix<double, 1, 3> rotatePixelXY;									//像素点旋转后坐标
	Matrix<double, 1, 2> srcPixelXY;									//像素点旋转前坐标
	bool pinkFlag, greenFlag, blueFlag, blackFlag, whiteFlag;			//各个颜色识别标志
	int wallDis = 0;													//墙的距离
	int accCount = 0;													//多周期确定

	float changeRoll;
};
#endif
