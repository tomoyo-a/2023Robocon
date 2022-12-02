//激光雷达头文件
#include "Urg_driver.h"
#include "Connection_information.h"
#include "urg_connection.h"
#include "sensor_parameter.h"
#include "map.h"
#include "pos.h"
#include "serial.h"
#include "pps.h"
#include "data.h"
#include <fstream>
#include <unistd.h>
#include <termio.h>
#include <atomic>
//opencv头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>

//进程通讯头文件
extern "C"
{
#include "process_comm.h"
}
#define WINSIZE 1000
using namespace std;
using namespace cv;

//判断雷达是否被遮挡
bool GetLiadrDis(int i);
void SigHandler(int signo);
bool GoodPoint(int i, Point P);
int PointOnWall(Point tmpPoint, vector<LineParam_t> allLine);
void CreateRadarImage(Mat image, vector<cv::Point> point, vector<cv::Point> &allImagePoint);
char *NameFileWithTime(string fileName);
int TimeCount_us(uint8_t index);
int totalPoint;
int lidarErrorFlag = 0;
bool InitialFlag = false;
bool InitPosFlag = true;

int ErrorMax = 700;
bool correctPosFlag = true;
vector<double> lineDistance;
double singleDistance;
LineParam_t getLineParam;
//关于场地的参数
vector<LineParam_t> wallLine; //外场的10条直线

vector<Point> pathPointRead;
extern pthread_mutex_t mtx;

//关于点的参数
vector<cv::Point> allImagePoint;
extern vector<LineParam_t> allCalculateLine;
ofstream output;
ofstream outFile;
ofstream path;
ofstream outTime;
ofstream outangle;

Robot_t robot;
Mat radarImage = Mat::zeros(WINSIZE, WINSIZE, CV_8UC3);
//int mcuSerial=0;
//USB初始化
int ppsSerial = 0;

atomic_bool correctionFlag;
atomic_bool runFlag;
chrono::steady_clock::time_point S1;
chrono::steady_clock::time_point M1;
Mat mapImage = Mat::zeros(WINSIZE, WINSIZE, CV_8UC3);
atomic_int proStamp;
Urg_driver urg;
int main(int argc, char *argv[])
{
	static int correctionflag = 0;
	//重新定义对应Ctrl+C对应的信号量处理函数
	signal(SIGINT, SigHandler);
#ifdef UNLIDA
#else
	/*检查是否能通过相应的串口连接激光雷达,重复尝试*/
	Connection_information information(argc, argv);
	//打开雷达
	// Connects to the sensor
	while (!urg.open(information.device_or_ip_name(),
					 information.baudrate_or_port_number(),
					 information.connection_type()))
	{
		Delay_ms(1000);
		cout << "Urg_driver::open(): "
			 << information.device_or_ip_name() << ": " << urg.what() << endl;
	}
	/*控制台显示当前设备信息*/
	PrintfDeviceInformation(urg);
	//启动测量
	urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
	cout << "-------------lidar ok-------------" << endl;
#endif
	//创建共享内存
	ProcessCommInit(MC_LIDAR_KEY_ID, LIDAR);
	own_serial ownSerial;
	//串口
	ownSerial.Init(ppsSerial, (char *)"/dev/ttyTHS0", B115200);
	//runFlag必须在开启串口线程前赋值为true
	correctionFlag = false;
	runFlag = true;
	//开启子线程用于串口收数
	thread serialThread(ReceiveData, ppsSerial);
	serialThread.detach();
	//等待定位系统初始化完成
	std::cout << "pps waiting" << endl;
	WaitPpsReady(ppsSerial);
	std::cout << "-------------pps ok-------------" << endl;

	//创建新窗口
	// cvNamedWindow("map", 1);
	//初始化场地图像
	InitMap();
	ShowMap(mapImage, wallLine);

	output.open("ldata.txt");
	outFile.open("lraw.txt");
	outangle.open("langle.txt");
	path.open("pathXY.txt");
	outTime.open("lTime.txt");
	cv::Point tmpPoint;
	vector<Pos_t> compenData;
	int codeTime[10] = {0};
	//矫正模式
	Delay_ms(200);
	robot.mode = CORRECTANGLE;
	while (runFlag)
	{
#ifdef UNLIDAR
#else
		TimeCount_us(0);
		int totalNum;
		//计时
		vector<unsigned short> intensity;
		long time_stamp = 0;
		proStamp++;
		cout << "proStamp: " << proStamp << endl;
		outangle << "proStamp: " << proStamp << endl;
		// outFile << proStamp << ":" << endl;
		outTime << "proStamp: " << proStamp << " ";
		pthread_mutex_lock(&mtx); //上锁
		robot.recentPosData.assign(robot.posData.begin(), robot.posData.end());
		pthread_mutex_unlock(&mtx); //解锁
		robot.ppsRawPos = robot.recentPosData.at(0);
		std::for_each(std::begin(robot.recentPosData), std::end(robot.recentPosData),
					  [&](Pos_t &pos)
					  { pos = pos + robot.ppsCorrect; });

		robot.rbotPos = robot.recentPosData.at(0);
		//清空储存数据容器
		DataClear();
		//获得数据 调用数据 数据储存在lidarData里
		TimeCount_us(6);
		while(1)
		{	
			if (!urg.get_distance_intensity(robot.lidarData, intensity, &time_stamp))
			{
				std::cout << "Urg_driver::get_distance(): " << urg.what() << endl;
				lidarErrorFlag++;
				Delay_ms(10);
			}
			else
			{
				lidarErrorFlag=0;
				break;
			}
			if(lidarErrorFlag>5)
			{
				lidarErrorFlag=0;
				urg.close();
				return 1;
			}
		}
		codeTime[6] = TimeCount_us(6);
		//往容器中存数
		totalPoint = robot.lidarData.size();
		//动态矫正
		DataCompensation(robot.recentPosData, compenData);
		// cout<<"flag1"<<endl;
		//计算坐标及角度
		int index = 0;

		float tmpTheta = 0;
		TimeCount_us(1);
		for (int i = 0; i < totalPoint; i++)
		{
			//角度-45~225,此时倒装反向，并转移到世界坐标系
			tmpTheta = 225 + robot.lidarRotation - (i / 4.0) + compenData[i].theta;
			LimitAng(tmpTheta);
			robot.radarDistance.push_back(robot.lidarData[i]);
			robot.radarTheta.push_back(tmpTheta * CV_PI / 180);
			tmpPoint.x = robot.radarDistance[i] * cos(robot.radarTheta[i]) + compenData[i].x;
			tmpPoint.y = robot.radarDistance[i] * sin(robot.radarTheta[i]) + compenData[i].y;
			/*------------------------测试雷达误差---------------------------*/
			// robot.radarTheta.push_back(((225 - (i / 4.0)) * CV_PI / 180));
			// tmpPoint.x = robot.radarDistance[i] * cos(robot.radarTheta[i]);
			// tmpPoint.y = robot.radarDistance[i] * sin(robot.radarTheta[i]);
			/*------------------------测试雷达误差---------------------------*/

#ifndef TEST_LIDAR
			if ((robot.lidarData[i] > 451) && (intensity[i] > -0.5 * robot.lidarData[i] + 3410.8))
#else
			if (robot.lidarData[i] > 451)
#endif
			{
				// if (robot.lidarData[i] > 820 && robot.lidarData[i] < 1250)
				// {
				// 	robot.lidarData[i] = robot.lidarData[i] * 1.0171 - 38.794;
				// }
				if (GoodPoint(i, tmpPoint))
				{
					robot.radarPoint.push_back(tmpPoint);
				}
				// outFile << i << ' ' << tmpPoint.x << ' ' << tmpPoint.y << ' ' <<robot.lidarData[i]<< endl;
			}
		}
		// //复制场地图像信息
		// radarImage = mapImage.clone();
		// //创建雷达扫描图像信息
		// CreateRadarImage(radarImage, robot.radarPoint, allImagePoint);
		codeTime[1] = TimeCount_us(1);
		totalNum = robot.radarPoint.size();
		//选择矫正模式
		switch (robot.mode)
		{
		case CORRECTANGLE:
		{
			//寻找角点
			TimeCount_us(2);
			cout << "flag1" << endl;
			DataProcess();
			cout << "flag1" << endl;
			codeTime[2] = TimeCount_us(2);
			//角点排序，以及直线拟合
			TimeCount_us(3);
			GetLine();
			codeTime[3] = TimeCount_us(3);
			cout << "findLineNume: " << allCalculateLine.size() << endl;
			TimeCount_us(4);
			/*------------------------测试雷达误差---------------------------*/
			// int lineNum = allCalculateLine.size();
			// for (size_t i = 0; i < lineNum; i++)
			// {
			// 	LineParam_t tmpLine = allCalculateLine.at(i);
			// 	int D = fabs(tmpLine.C) / sqrt(pow(tmpLine.A, 2) + pow(tmpLine.B, 2));
			// 	// CompenDistance(D);
			// 	// CompenStartDis(D);
			// 	allCalculateLine.at(i).rbot2Line = D;
			// 	cout << allCalculateLine.at(i) << endl;
			// }
			/*------------------------测试雷达误差---------------------------*/
			if (CorrectAngle(allCalculateLine, 12))
			{
				robot.mode = CORRECTPOS;
			}
			codeTime[4] = TimeCount_us(4);
			break;
		}
		case CORRECTPOS:
		{
			cout << "---------------------------------------------" << endl;

			// if (correctionFlag && 0)
			// {
			// 	ScreenRadarPoint();
			// }
			// else
			{
				TimeCount_us(2);
				//寻找角点
				DataProcess();
				codeTime[2] = TimeCount_us(2);
				TimeCount_us(3);
				//角点排序，以及直线拟合
				GetLine();
				codeTime[3] = TimeCount_us(3);
				// cout<<"flag1"<<endl;
				TimeCount_us(4);
				//地图匹配
				MatchMap();
				// cout<<"flag2"<<endl;

				codeTime[4] = TimeCount_us(4);
			}
			correctionflag++;
			// 当TR在出发区时补偿x方向距离
			//  && (robot.rbotPos.y > 800 && robot.rbotPos.x > 800)
			if (correctionflag == 60)
			{
				correctionFlag = true;
				// correctPosFlag = false;
				InitialFlag = true;
				ErrorMax = 300;
			}
			// if((robot.rbotPos.x > 2200 && robot.rbotPos.x < 2300) && (robot.rbotPos.y > 9150 && robot.rbotPos.y < 9300))
			// {
			// 	if((fabs(robot.ppsCorrect.x) < 3 && fabs(robot.ppsCorrect.y) < 3) && CorrectionAgain)
			// 	{
			// 		if(fabs(robot.rbotVel.x)<2 && fabs(robot.rbotVel.y) < 2)
			// 		{
			// 			robot.mode = CORRECTANGLE;
			// 			CorrectionAgain = false;
			// 		}
			// 	}
			// }
			cout << "R " << robot.rbotPos << endl;
			//更新是否要进行矫正
			// correctPosFlag = SetPosFlag();
			// cout << "correctPosFlag: " << correctPosFlag << endl;
			// output << "correctPosFlag: " << correctPosFlag << endl;

			break;
		}
		default:
			break;
		}
		// imshow("map", radarImage);
		// //必须有这一句，否则无法正常显示图片
		// cv::waitKey(1);
		allImagePoint.clear();
		TimeCount_us(5);
		robot.OutputInformation(output);
		codeTime[5] = TimeCount_us(5);
		codeTime[0] = TimeCount_us(0);
		for (uint8_t i = 0; i < 10; i++)
		{
			outTime << codeTime[i] << ' ';
		}
		outTime << robot.serialFlag << endl;
#endif
	}

	output.close();
	path.close();
	outangle.close();
	urg.close();
	cout << "关闭URG: " << urg.what() << endl;

	waitKey(0);
}
//按键产生的信号量对应的处理函数
//ctrl+c 退出程序
void SigHandler(int signo)
{
	static char exitFlag = 0;
	cout << signo << endl;
	switch (exitFlag)
	{
	case 2:
		runFlag = 0;
		urg.close();
		cout << "关闭URG: " << urg.what() << endl;
		break;
	case 3:
		exit(0);
		break;
	default:
		if (exitFlag > 4)
		{
			exitFlag = 0;
		}
		break;
	}
	exitFlag++;
}
/** @brief 绘制旋转矩形（默认世界坐标系为最常见的普通正交坐标系）
 * @param img Image
 * @param center 矩形在图像坐标系中的中心点坐标
 * @param angle 该矩形相对于世界坐标系旋转角度，逆时针为正
 * @param color 颜色
 * @param len 长
 * @param width 宽
*/
void DrawRect(InputOutputArray img, cv::Point center, float angle, const cv::Scalar &color, int len = 30, int width = 30)
{
	//矩形的四个顶点
	int dx = len / 2;
	int dy = width / 2;
	vector<cv::Point> recVertexes(4);
	cv::Point tmpPoint;
	//从矩形坐标系中逆变换到世界坐标系，故角度取反
	angle = ANGTORAD(-angle);
	//将矩形顶点从矩形坐标系变换到图像坐标系
	tmpPoint = DataTransform(cv::Point(-dx, dy), 0, 0, angle);
	recVertexes[0] = cv::Point(center.x + tmpPoint.x, center.y - tmpPoint.y);
	tmpPoint = DataTransform(cv::Point(dx, dy), 0, 0, angle);
	recVertexes[1] = cv::Point(center.x + tmpPoint.x, center.y - tmpPoint.y);
	tmpPoint = DataTransform(cv::Point(dx, -dy), 0, 0, angle);
	recVertexes[2] = cv::Point(center.x + tmpPoint.x, center.y - tmpPoint.y);
	tmpPoint = DataTransform(cv::Point(-dx, -dy), 0, 0, angle);
	recVertexes[3] = cv::Point(center.x + tmpPoint.x, center.y - tmpPoint.y);
	//绘制矩形
	cv::polylines(img, recVertexes, 1, color, 1);
}
void CreateRadarImage(Mat image, vector<cv::Point> point, vector<cv::Point> &allImagePoint)
{
#ifndef TEST_LIDAR
	Pos_t rPosData = robot.rbotPos;
#else
	Pos_t rPosData = robot.recentPosData[0];
#endif
	int x, y;
	u_char *index = 0;
	//绘制车体方向指示线
	cv::Point pt1, pt2;
	pt1 = P2Img(cv::Point(rPosData.x, rPosData.y));
	float rAng = rPosData.theta + robot.ppsInitAngle;
	pt2 = P2Img(cv::Point(rPosData.x + 800 * cos(ANGTORAD(rAng)), rPosData.y + 800 * sin(ANGTORAD(rAng))));
	arrowedLine(image, pt1, pt2, Scalar(34, 139, 34));
	//绘制车体
	DrawRect(image, pt1, rPosData.theta, Scalar(0, 55, 205));
	path << robot.rbotPos.x << '\t' << robot.rbotPos.y << endl;
	cv::Point pt = P2Img(cv::Point(rPosData.x, rPosData.y));
	cv::circle(mapImage, pt, 1, Scalar(37, 193, 255), -1);
	//雷达扫描点
	cv::Point imagePoint;
	for (int i = 0; i < point.size(); i++)
	{
		imagePoint = P2Img(point[i]);
		cv::circle(radarImage, imagePoint, 1, Scalar(37, 193, 255), -1);
		x = imagePoint.x;
		y = imagePoint.y;
		allImagePoint.push_back(imagePoint);
	}
}

char *NameFileWithTime(string fileName)
{
	char str1[20];
	char str2[20];
	time_t now = time(NULL);
	//读取时间并转化为字符串
	strftime(str1, 5, "%m%d", localtime(&now));
	strftime(str2, 5, "%H%M", localtime(&now));
	string tmp = str1;
	tmp.append(str2);
	tmp.append(fileName);
	tmp.append(".txt");
	strcpy(str1, tmp.data());
	char *ret = new char[strlen(str1) + 1];
	strcpy(ret, str1);
	return ret;
}

bool GoodPoint(int i, Point P)
{
	if ((i < 820 || i > 856) && (i < 230 || i > 380) && (i < 484 || i > 621))
	{
		if (P.x < 510 && P.y > 10300)
		{
			return 0;
		}
		else if (abs(P.x - 1975) < 35 && abs(P.y - 6975) < 15)
		{
			return 0;
		}
		else if ((abs(P.x - 3450) < 165 || abs(P.x - 5950) < 165 || abs(P.x - 8450) < 165) && abs(P.y - 5950) < 35)
		{
			return 0;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}
/*程序段运行时间计时
 *@param index 计时指向
 *@return 时间差值 单位us
 */
int TimeCount_us(uint8_t index)
{
	//开始计数标志
	static bool isCountFlag[10] = {0};
	static struct timespec start_time[10] = {0};
	if (!isCountFlag[index])
	{
		//更新开始计时的时间
		clock_gettime(CLOCK_REALTIME, &start_time[index]);
		isCountFlag[index] = !isCountFlag[index];
		return -1;
	}
	else
	{
		struct timespec end_time;
		clock_gettime(CLOCK_REALTIME, &end_time);
		int diffTime = ((end_time.tv_sec * pow(10, 9) + end_time.tv_nsec) - (start_time[index].tv_sec * pow(10, 9) + start_time[index].tv_nsec)) / pow(10, 3);
		isCountFlag[index] = !isCountFlag[index];
		return diffTime;
	}
}