#include "act_L515.h"

extern std::mutex uptateMutex;
extern std::map<int, int> counters;
extern std::map<int, std::string> stream_names;

int main(int argc, char* argv[])
{
	// #ifdef TXT//根据系统时间生成TXT文件
    // //获取系统时间戳
    // time_t timeReal;
    // time(&timeReal);
    // timeReal = timeReal + 8*3600;//	格林尼治标准时间+8个小时
    // tm* t = gmtime(&timeReal);

    // ofstream Fout("../cvData/" + format("%.2d%.2d\n%.2d:%.2d", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min) + "\nrecord.txt");
	// Fout << "mainTask" << "\t" << "subTask" << "\t" << "rollAngle" << "\t" << "pitchAngle" << "\t" << "yawAngle" << "\t" 
	// 	<< "left"	<< "\t" << "right" << "\t" << "distance" << "\t" << "Angle" << "\t" << "timeFlag" << "\t" << "status" << "\t" << "time" << endl;
    // #endif

	// #ifdef RECORD_VIDEO
    // VideoWriter writer("../cvVideos/" + format("%.2d%.2d\n%.2d:%.2d", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min) + "\ndst.avi", VideoWriter::fourcc('P', 'I', 'M', '1'), 30.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
    // #endif

	// own_serial ownSerial("/dev/ttyUSB0",B115200);//通信模块实例化
	ActL515 L515;                                //相机驱动模块实例化

	// pPointCloud		outCloud(new pointCloud), dstCloud(new pointCloud);//存放显示的点云
	cv::Mat outImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);//存放显示的图像

	//init
	L515.Init();

	#ifdef COMMUN
	ProcessCommInit(MC_CV_ID, CV);
	#endif

	char task = 0;
	bool timeFlag = 0;
	char status;        //与运动控制通信时便于视觉计算周期, 信息有效标志位

	cv::TickMeter tk;

	while (true)
	{
		// std::lock_guard<std::mutex> lock(uptateMutex);

		// std::cout << std::endl;
		// for (auto p : counters)
		// {
		// 	std::cout << stream_names[p.first] << "[" << p.first << "]: " << p.second << " [frames] || ";
		// }
	}


	return EXIT_SUCCESS;
}

