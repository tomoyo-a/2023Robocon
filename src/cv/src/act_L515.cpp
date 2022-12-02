#include "act_L515.h"

/********************************IMU****************************************/
// theta is the angle of camera rotation in x, y and z components
float3 theta;						//相机IMU坐标系为右手系，z轴指向相机背后，相机水平正向放置时,重力加速度指向Y轴负方向
float3 initialTheta;
float3 thetaCameraChange;

std::mutex thetaMtx;
/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
float alpha = 0.995f;
int initGyro = 0;
int firstAccel = 5;
int firstFrameSet = 5;
// Keeps the arrival time of previous gyro frame
double last_ts_gyro = 0;
float rollCameraChange;                                             //

std::mutex uptateMutex;
std::mutex communMutex;
std::map<int, int> counters;
std::map<int, std::string> stream_names;

ofstream Tout("../cvData/tempData.txt");
ofstream Gout("../cvData/gyroData.txt");
ofstream Aout("../cvData/accelData.txt");
VideoWriter writer;

ActL515::ActL515():
alignToColor(RS2_STREAM_COLOR)
{    
	data = new unsigned short[640 * 480];
}

void ActL515::Init(void)
{
	#ifndef IFCAMERA
	cfg.enable_device_from_file("../record/SeeSaw.bag");
	#else
	//pipe.stop();
	// if (!check_imu_is_supported())
    // {
    //     std::cerr << "Device supporting IMU not found";
    //     return;
    // }
	auto devices = ctx.query_devices();//获取设备列表
	device_count = devices.size();					//获取传感器连接数量
	cout << "device_count  " << device_count << endl;
	while (!device_count)							//持续访问，直到有设备连接为止
	{
		devices = ctx.query_devices();
		device_count = devices.size();
		cout<<"device_count:"<< device_count<<endl;
		cout <<"No device detected. Is it plugged in?\n";
		//return;
	}
	// Get the first connected device
	auto dev = devices[0];

	std::ifstream file("../src/cv/modeJson/imu.json");  //加载相机参数设置文件，保存相对路径
	if (file.good())
	{
		std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

		auto prof = cfg.resolve(pipe);
		if (auto advanced_mode_dev = prof.get_device().as<rs2::serializable_device>())
		{
			advanced_mode_dev.load_json(str);
		}
		else
		{
			cout << "Current device doesn't support advanced-mode!\n";
			return;
		}
	}
	cout << "reading setting mode is OK" << endl; 

	cfg.disable_all_streams();												//失能所有数据流
    cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 30);      //使能深度相机输入流
	// //配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
	cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 30);		//使能彩色相机输入流
	// cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF, 30);
	cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);		//使能角速度流
	cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 400);		//使能加速度流
	#endif

	// Define frame callback
    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
	auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(uptateMutex);
        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            for (const rs2::frame& f : fs)
                counters[f.get_profile().unique_id()]++;

            // Handle image
            // TickMeter tk;
            // tk.start();

            Update(fs);

            #ifdef COMMUN
			getMCmessage();
            #endif

            TickMeter tk;
            tk.start();
			imagePreProcess();
            SortColorNew(filed, ballOrBucket);
			
            #ifdef DEBUG
                showImage();
            #endif

            #ifdef COMMUN
                SendMessage();
            #endif

            recordData();

			release();

            tk.stop();
            cout << "-----------------total time:" << tk.getTimeMilli() << endl << endl;

            #ifdef TXT
            Fout << tk.getTimeMilli() << endl;
            #endif
        }
        else if (rs2::motion_frame motionFrame = frame.as<rs2::motion_frame>())
        {
            if (motionFrame.get_profile().stream_name() == "Gyro")
            {
				// Get the timestamp of the current frame
            	double ts = motionFrame.get_timestamp();
                //Handle Gyro data
				rs2_vector gyroData = motionFrame.get_motion_data();
				processGyro(gyroData, ts);
                float gyroModule = sqrtf(powf(gyroData.x, 2.0) + powf(gyroData.y, 2.0) + powf(gyroData.z, 2.0));
				Gout << "gyroData:" << "\t" << gyroData.x << "\t" << gyroData.y << "\t" << gyroData.z << "\t" << gyroModule << endl;

               counters[frame.get_profile().unique_id()]++;
            }
            else if (motionFrame.get_profile().stream_name() == "Accel") 
            {
                //Handle Accel data
                rs2_vector accelData = motionFrame.get_motion_data();
				processAccel(accelData);
                float accelModule = sqrtf(powf(accelData.x, 2.0) + powf(accelData.y, 2.0) + powf(accelData.z, 2.0));
				Aout << "accelData:" << "\t" << accelData.x << "\t" << accelData.y << "\t" << accelData.z << "\t" << accelModule << endl;

                counters[frame.get_profile().unique_id()]++;
            }
        }
    };

	// Start streaming through the callback with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    //
	rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    #ifdef IFCAMERA
    auto colorSensors = profiles.get_device().query_sensors()[1];
	colorSensors.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0);                              //禁用曝光优先级
	// colorSensors.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0); //zero is turn off
	// colorSensors.set_option(RS2_OPTION_EXPOSURE, 650);
	// colorSensors.set_option(RS2_OPTION_GAIN, 300);    										//设置曝光时间
	// colorSensors.set_option(RS2_OPTION_WHITE_BALANCE, 4500);									//设置白平衡
	cout << "set exposure OK ..\n";

	auto depthSensors = profiles.get_device().query_sensors()[0];
	depthSensors.set_option(RS2_OPTION_LASER_POWER, 100);

    auto device = profiles.get_device();
    for (auto& sensor : device.query_sensors()) 
    {
        sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0);
        // Configuring other options here, e.g. disabling emitter, works as expected regardless of call to enable_device()
    }
	#endif

    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        stream_names[p.unique_id()] = p.stream_name();

    std::cout << "RealSense callback sample" << std::endl << std::endl;

	/********OpenCv Init********/
	imgCenter = Point2f((IMAGE_WIDTH - 1) / 2.f, (IMAGE_HEIGHT - 1) / 2.f);

	srcImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
    HSVImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3);
    depthImage = cv::Mat::zeros(cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_16UC1);

    #ifdef TXT
    time(&timeReal);
    timeReal = timeReal + 8*3600;//	格林尼治标准时间+8个小时
    t = gmtime(&timeReal);

    Fout = ofstream("../cvData/" + format("%.2d%.2d\n%.2d:%.2d", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min) + "\nrecord.txt");
    Fout << "timeFlag" << "\t" << "filed" << "\t" << "ballOrBucket" << "\t" << "baffleOrLeft" << "\t" << "robotX" << "\t" << "robotRoll" << "\t" << "robotYaw" << "\t" 
		 << "colorFlag" << "\t" << "locateFlag" << "\t" << "lBaffleFlag" << "\t" << "sideWallCameraX" << "\t"
		 << "colorMsg1" << "\t" << "colorMsg2" << "\t" << "colorMsg3" << "\t" << "colorMsg4" << "\t" << "colorMsg5" << "\t" 
		 << "ballBucketAngle" << "\t" << "ballBucketDis" << "\t" << "baffleAngle" << "\t" << "lBaffleDis" << "\t" << "wallYawAngle" << "\t" << "time" 
		 << endl;
    #endif

    #ifdef RECORD_VIDEO
    writer = VideoWriter("../cvVideos/" + format("%.2d%.2d\n%.2d:%.2d", t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min) + "\ndst.avi", VideoWriter::fourcc('P', 'I', 'M', '1'), 30.0, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
    #endif
}

void ActL515::Update(rs2::frameset fs)
{
    TickMeter tk;
    tk.start();

	alignProcess = alignToColor.process(fs);
	rs2::video_frame colorFrame = alignProcess.get_color_frame();				//获取彩色帧
	rs2::depth_frame alignedDepthFrame = alignProcess.get_depth_frame();

	srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);				//将彩色图像数据存储在Mat矩阵中
	depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)alignedDepthFrame.get_data(), cv::Mat::AUTO_STEP);	//将深度图像数据存储在Mat矩阵中

	if(firstFrameSet >= 0)
	{
		firstFrameSet--;
		getCameraExtrinsics(alignProcess);
	}

    timeFlag = !timeFlag;

    tk.stop();
    cout << "update time:" << tk.getTimeMilli() << endl;
	return;
}

void ActL515::imagePreProcess(void)
{
    float changeRoll;
    static float tempRoll = 0.f;
    // static int frameNum = 0;
	if(1)
	{
		std::lock_guard<std::mutex> lock(thetaMtx);
        //二维旋转部分
        changeRoll = thetaCameraChange.x;
        tempRoll += 10.f;
        cout << "changeRoll:" << changeRoll << endl;
	}
	// cout << "rotatetion:" << theta.x << "\t" << theta.y << "\t" << theta.z << endl;
	// cout << "thetaCameraChange:" << thetaCameraChange.x << "\t" << thetaCameraChange.y << "\t" << thetaCameraChange.z << endl; 

	// adjust transformation matrix
    TickMeter tk;
    tk.start();

    cout << "robotRoll:" << robotRoll << endl;

    // robotRoll = robotRoll + 1.5f;                                       //加补偿，使陀螺仪与相机的零度一致
    robotRoll = -robotRoll;                                                //使角度为左正右负
    //三维点旋转向量
    rotateVector = AngleAxisd(-robotRoll / 180.f * CV_PI, Vector3d(0, 0, 1));//逆时针为正
    if(filed == RED_FIELD && ballOrBucket == BALL)
    {
        robotRoll = robotRoll + 0.4f;
    }
    else if(filed == BLUE_FIELD && ballOrBucket == BALL)
    {
        robotRoll = robotRoll + 0.8f;
    }
    else if(filed == BLUE_FIELD && ballOrBucket == BUCKET)
    {
        robotRoll = robotRoll - 2.f;
    }


    Rect bbox;
    imageRotationMatix = getRotationMatrix2D(imgCenter, robotRoll, 1.0);        //逆时针为正
    bbox = RotatedRect(imgCenter, srcImage.size(), robotRoll).boundingRect();
    imageRotationMatix.at<double>(0, 2) += bbox.width / 2.0 - imgCenter.x;
    imageRotationMatix.at<double>(1, 2) += bbox.height / 2.0 - imgCenter.y;
	warpAffine(srcImage, rotateImage, imageRotationMatix, bbox.size());
    warpAffine(depthImage, rotateDepthImage, imageRotationMatix, bbox.size());
    invertAffineTransform(imageRotationMatix, tempRotationMatix);
    cv2eigen(tempRotationMatix, imageInvRotateMatrix);

    tk.stop();
    cout << "rotate time:" << tk.getTimeMilli() << endl;

    dstImage = rotateImage.clone();
    cv::cvtColor(rotateImage, HSVImage,cv::COLOR_BGR2HSV);

    // Vector3d tempPoint = rotateVector * Vector3d(1.0, 0.f, 0.f);

    // cout << "tempPoint:" << tempPoint[0] << " " << tempPoint[1] << " " << tempPoint[2] << endl;

    // frameNum++;
    // string picName = "../cvFrames/" + std::to_string(frameNum) + ".jpg";
    // cv::imwrite(picName, rotateImage);
    // picName = "../cvSrcImg/" + std::to_string(frameNum) + ".jpg";
    // cv::imwrite(picName, srcImage);
}

bool ActL515::SortColorNew(char field, char task)
{
    static int total = 0;
    static int rightTime = 0;
    static char firstColor = 6;
    static char secondColor = 6;

    vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
    vector<TargetOutPut> targetVector;

    Mat bin;                        //中间图像
    int targetBeginX;               //roi的X起始点
    int targetWidth;                //roi的X宽度
    Rect roi;                       //设置roi
    Rect targetRect;                //用来框墙
    double area;                    //轮廓面积
    float targetInterval;           //球或桶两两间距
    float firstTargetBeginX;        //第一个球或桶距离左侧墙的距离
    float targetDepth;              //目标点至球或桶中心深度
    int tempWallDis;
    Point2i tempCenter;
    Scalar colorLow_L, colorUp_L;
    Scalar colorLow_R, colorUp_R;
    float angleCompensation;        //目标补偿的角度
    float XCompensation;        //目标补偿的角度

    switch(task)//根据不同场和任务识别墙
    {
        case BALL:
            firstTargetBeginX = 0.250;
            targetInterval = 0.375;
            robotYaw = robotYaw + 90.f;//偏航角初始方向为-90，逆时针为正,这里统一转换为车的前进方向左正右负
            targetDepth = 0.0215f;//球的直径为42-43mm
            if(filed == BLUE_FIELD)
            {
                cout << "now is BLUE_FIELD ball" << endl;
                angleCompensation = 1.26038f;
                sideWallCameraX = -(robotX / 1000.f - sinf(robotYaw / 180.f * CV_PI) * ROBOTCENTER_Z - sinf(robotRoll / 180.f * CV_PI) * fabs(cosf(robotYaw / 180.f * CV_PI)) * ROBOTCENTER_Y);//m
            } 
            else if(filed == RED_FIELD)
            {
                cout << "now is RED_FIELD ball" << endl;
                angleCompensation = 1.6f;
                sideWallCameraX = -(2.f + (robotX / 1000.f - sinf(robotYaw / 180.f * CV_PI) * ROBOTCENTER_Z - sinf(robotRoll / 180.f * CV_PI) * fabs(cosf(robotYaw / 180.f * CV_PI)) * ROBOTCENTER_Y));//m
            }
            targetBeginX = 320;
            targetWidth = 80;
            colorLow_L = Scalar(BALLWALL_MIN_H_L, BALLWALL_MIN_S, BALLWALL_MIN_V);
            colorUp_L  = Scalar(BALLWALL_MAX_H_L, BALLWALL_MAX_S, BALLWALL_MAX_V);
            colorLow_R = Scalar(BALLWALL_MIN_H_R, BALLWALL_MIN_S, BALLWALL_MIN_V);
            colorUp_R  = Scalar(BALLWALL_MAX_H_R, BALLWALL_MAX_S, BALLWALL_MAX_V);
            break;
        case BUCKET:
            targetInterval = -0.250;
            robotYaw = robotYaw - 90.f;
            targetDepth = 0.050f;//桶的直径为100mm
            if(field == BLUE_FIELD)
            {
                cout << "now is BLUE_FIELD BUCKET " << endl;
                angleCompensation = 1.26038f;
                firstTargetBeginX = -0.750;
                sideWallCameraX = (robotX / 1000.f + sinf(robotYaw / 180.f * CV_PI) * ROBOTCENTER_Z + sinf(robotRoll / 180.f * CV_PI) * fabs(cosf(robotYaw / 180.f * CV_PI)) * ROBOTCENTER_Y);//m
                targetBeginX = 0;
                targetWidth = HSVImage.cols;
                colorLow_L = Scalar(BLUEWALL_MIN_H_L, BLUEWALL_MIN_S, BLUEWALL_MIN_V);
                colorUp_L  = Scalar(BLUEWALL_MAX_H_L, BLUEWALL_MAX_S, BLUEWALL_MAX_V);
                colorLow_R = Scalar(BLUEWALL_MIN_H_R, BLUEWALL_MIN_S, BLUEWALL_MIN_V);
                colorUp_R  = Scalar(BLUEWALL_MAX_H_R, BLUEWALL_MAX_S, BLUEWALL_MAX_V);
            }
            else if(field == RED_FIELD)
            {
                cout << "now is RED_FIELD BUCKET " << endl;
                angleCompensation = 1.6f;
                firstTargetBeginX = -0.250;
                sideWallCameraX = 2.f + (robotX / 1000.f + sinf(robotYaw / 180.f * CV_PI) * ROBOTCENTER_Z + sinf(robotRoll / 180.f * CV_PI) * fabs(cosf(robotYaw / 180.f * CV_PI)) * ROBOTCENTER_Y);
                targetBeginX = 280;
                targetWidth = 80;
                colorLow_L = Scalar(REDWALL_MIN_H_L, REDWALL_MIN_S, REDWALL_MIN_V);
                colorUp_L  = Scalar(REDWALL_MAX_H_L, REDWALL_MAX_S, REDWALL_MAX_V);
                colorLow_R = Scalar(REDWALL_MIN_H_R, REDWALL_MIN_S, REDWALL_MIN_V);
                colorUp_R  = Scalar(REDWALL_MAX_H_R, REDWALL_MAX_S, REDWALL_MAX_V);
            }
            break;
        default:
            break;
    }

    roi = Rect(targetBeginX, 0, targetWidth, HSVImage.rows);
    bin = Mat(HSVImage, roi).clone();
    ImageDepthFilter(bin, roi, 300, 3050, 255);//第一次深度筛选
    // imshow("wall", bin);
    Mat bin1, bin2;
    inRange(bin, colorLow_L, colorUp_L, bin1);//墙二值化
    inRange(bin, colorLow_R, colorUp_R, bin2);//墙二值化
    addWeighted(bin1, 1.0, bin2, 1.0, 0, bin);
    // imshow("bin", bin);
    findContours(bin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//寻找墙的轮廓
    if(contours.size() == 0)
    {
        cout << "no contours " << endl;
        return false;
    }
    sort(contours.begin(), contours.end(), compContours);//按面积大小从大到小排序
    area = contourArea(contours[0]);
    if(area > 500)
    {
        area = 0.f;
    }
    else
    {
        cout << "area not enough" << endl;
        return false;
    }
    targetRect = boundingRect(contours[0]) + Point(targetBeginX, 0);
    tempCenter = Point2i(targetRect.tl().x / 2 + targetRect.br().x / 2, targetRect.tl().y + targetRect.height * 3 / 4);
    
    rectangle(dstImage, targetRect, Scalar(0, 0, 255), 1, 8);
    circle(dstImage, tempCenter, 2, Scalar(0, 255, 255), 1, 8);
    
    tempWallDis = rotateDepthImage.ptr<ushort>(tempCenter.y)[tempCenter.x];
    if(task == BUCKET && field == BLUE_FIELD)
    {
        targetBeginX = targetRect.tl().x + 9.f * targetRect.width / 10;//第二次精选确定蓝墙高度roi;
        targetWidth = targetRect.width / 10;
    }
    roi = Rect(targetBeginX, 0, targetWidth, HSVImage.rows);
    bin = Mat(HSVImage, roi).clone();
    ImageDepthFilter(bin, roi, tempWallDis / 5, tempWallDis / 4 + 30, 0);
    // imshow("wall 2", bin);
    inRange(bin, colorLow_L, colorUp_L, bin1);//墙二值化
    inRange(bin, colorLow_R, colorUp_R, bin2);//墙二值化
    addWeighted(bin1, 1.0, bin2, 1.0, 0, bin);

    vector<vector<Point>>().swap(contours);
    vector<Vec4i>().swap(hierarchy);

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(bin, bin, MORPH_OPEN, kernel);
    morphologyEx(bin, bin, MORPH_CLOSE, kernel); 
    //摆放区和分类区的墙
    findContours(bin, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//寻找墙的轮廓
    if(contours.size() == 0)
    {
        return false;
    }
    sort(contours.begin(), contours.end(), compContours);//按面积大小从大到小排序
    area = contourArea(contours[0]);
    cout << "wall area :" << area << endl;
    if(area > 200)
    {
        area = 0.f;
    }
    else
    {
        return false;
    }
    targetRect = boundingRect(contours[0]) + Point(targetBeginX, 0);

    /**************这段是取墙上目标区域上的两个点确定向量计算出偏航角****************/
    //这段是取墙上目标区域上的两个点确定向量计算出偏航角,运控那边用不用另说
    Point3f middleRight, middleLeft;
    Vector3d rightPoint, leftPoint;
    float realPoint[3];
    float depthPixel[2];
    int col1 = targetRect.tl().x + targetRect.width * 3 / 4;
    int row1 = targetRect.tl().y / 2 + targetRect.br().y / 2;
    // depthPixel[0] = invRotationMatix.at<double>(0, 0) * (float)col1 + invRotationMatix.at<double>(0, 1) * (float)row1 + invRotationMatix.at<double>(0, 2);
    // depthPixel[1] = invRotationMatix.at<double>(1, 0) * (float)col1 + invRotationMatix.at<double>(1, 1) * (float)row1 + invRotationMatix.at<double>(1, 2);
    rotatePixelXY << col1, row1, 1.f;
    srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
    depthPixel[0] = round(srcPixelXY(0,0));
    depthPixel[1] = round(srcPixelXY(0,1));
    float depth = rotateDepthImage.ptr<ushort>(row1)[col1] / 4000.f;//换算为m

    rs2_deproject_pixel_to_point(realPoint , &color_intrin, depthPixel ,depth);//2d 到 3d

    rightPoint = rotateVector * Vector3d(realPoint[0], realPoint[1], realPoint[2]);
    
    int col2 = targetRect.tl().x + targetRect.width / 4;
    int row2 = targetRect.tl().y / 2 + targetRect.br().y / 2;
    // depthPixel[0] = invRotationMatix.at<double>(0, 0) * (float)col2 + invRotationMatix.at<double>(0, 1) * (float)row2 + invRotationMatix.at<double>(0, 2);
    // depthPixel[1] = invRotationMatix.at<double>(1, 0) * (float)col2 + invRotationMatix.at<double>(1, 1) * (float)row2 + invRotationMatix.at<double>(1, 2);
    rotatePixelXY << col2, row2, 1.f;
    srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
    depthPixel[0] = round(srcPixelXY(0,0));
    depthPixel[1] = round(srcPixelXY(0,1));
    depth = rotateDepthImage.ptr<ushort>(row2)[col2] / 4000.f;//换算为m

    rs2_deproject_pixel_to_point(realPoint , &depth_intrin, depthPixel ,depth);//2d 到 3d

    leftPoint = rotateVector * Vector3d(realPoint[0], realPoint[1], realPoint[2]);

    line(dstImage, Point(col1, row1), Point(col2, row2), Scalar(0, 0, 255), 1, 8);

    Eigen::Vector3d zNormal(0, 0, 1);
    Eigen::Vector3d vecNormal = Eigen::Vector3d(rightPoint[0] - leftPoint[0], 0, rightPoint[2] - leftPoint[2]);//平面单位法向量

    float cosValue = vecNormal.dot(zNormal) / (vecNormal.norm() * zNormal.norm());
    wallYawAngle = (acos(cosValue) - CV_PI / 2) / CV_PI * 180.f;

    cout << "wallYawAngle:" << wallYawAngle << endl;
    /******************************/

    int targetBeginY;//这两个是确定目标线的Y
    float lowRadiu;//给线段长度设置下线 (本来是最小包围圆的半径下限)
    float upRadiu;
    Point2i targetCenter;//墙目标区域的中心,这这个点距离进行深度滤波,墙后150为距离阈值

    switch(task)//根据不同墙体确定目标区域和筛选阈值
    {
        case BALL:
            targetBeginY = (int)(targetRect.tl().y - 20.5f / 100.f * targetRect.height);
            lowRadiu = targetRect.height * 18.f/ 100.f;
            upRadiu = targetRect.height * 50.f / 100.f;
            targetCenter = Point2i(targetRect.tl().x / 2 + targetRect.br().x / 2, targetRect.tl().y / 2 + targetRect.br().y / 2);
            wallDis = rotateDepthImage.ptr<ushort>(targetCenter.y)[targetCenter.x];
            break;
        case BUCKET:
            if(field == BLUE_FIELD)
            {
                targetBeginY = (int)(targetRect.tl().y - 35.f / 100.f * targetRect.height);
                lowRadiu = targetRect.height * 20.f / 100.f;
                upRadiu = targetRect.height * 125.f / 100.f;
                targetCenter = Point2i(targetRect.tl().x / 2 + targetRect.br().x / 2, targetRect.tl().y / 2 + targetRect.br().y / 2);
                wallDis = rotateDepthImage.ptr<ushort>(targetCenter.y)[targetCenter.x];
            }
            else if(field == RED_FIELD)
            {
                targetBeginY = (int)(targetRect.tl().y - 44.f / 100.f * targetRect.height);
                lowRadiu = targetRect.height * 20.f / 100.f;
                upRadiu = targetRect.height * 125.f / 100.f;
                targetCenter = Point2i(targetRect.tl().x / 2 + targetRect.br().x / 2, targetRect.tl().y / 2 + targetRect.br().y / 2);
                wallDis = rotateDepthImage.ptr<ushort>(targetCenter.y)[targetCenter.x]; 
            }
            break;
        default:
            cout << "No this wall !!!!!" << endl;
            return false;         
    }
    rectangle(dstImage, targetRect, Scalar(0, 255, 0), 1, 8);
    circle(dstImage, targetCenter, 2, Scalar(0, 255, 0), 1, 8);

    cout << "wallDis" << wallDis / 4 << endl;//这里的÷4是因为L515的深度值是实际的4倍
    if(targetBeginY <= 0 || targetBeginY >= rotateImage.rows || wallDis == 0)//防止越界
    {
        cout << "find wall error !!!!!!!!!!" << endl;
        cout << "targetBeginY:" << targetBeginY << endl;
        return false;
    }

    vector<vector<Point>>().swap(contours);
    
    /************8for record video*********/
    rotatePixelXY << 0, (int)targetBeginY, 1.f;
    srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
    Point pointBegin = Point(round(srcPixelXY(0,0)), round(srcPixelXY(0,1)));

    rotatePixelXY << rotateImage.cols, (int)targetBeginY, 1.f;
    srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
    Point pointEnd = Point(round(srcPixelXY(0,0)), round(srcPixelXY(0,1)));

    // line(srcImage, pointBegin, pointEnd, Scalar(255, 255, 255), 1, 8);
    /*****************************************/
    
    bool lineBegin = false;
    int lineLength = 0;
    vector<Point> tempContour;
     //这段是在目标线上寻找球或桶
    line(dstImage, Point(0, (int)targetBeginY), Point(rotateImage.cols, (int)targetBeginY), Scalar(255, 255, 255), 1, 8);
    // line(srcImage, Point(0, 240), Point(640, 240), Scalar(0, 255, 0), 1, 8);
    // line(srcImage, Point(0, 245), Point(640, 245), Scalar(0, 255, 0), 1, 8);
    // line(srcImage, Point(0, 250), Point(640, 250), Scalar(0, 255, 0), 1, 8);
    // line(srcImage, Point(0, 255), Point(640, 255), Scalar(0, 255, 0), 1, 8);
    // line(srcImage, Point(0, 280), Point(640, 280), Scalar(0, 255, 0), 3, 8);
    for (int col = 0, row = (int)(targetBeginY); col < rotateImage.cols; col++)//根据深度筛选点,连续则放到同一个contour里
    {  
        if(!lineBegin && !depthJudge(col-1, row) && depthJudge(col, row) && depthJudge(col+1, row))//线段起点判断
        {
            tempContour.push_back(Point(col, row));
            lineBegin = true;
            circle(dstImage, Point(col, row), 1, Scalar(0, 0, 255), 1, 8);
        }
        else if(lineBegin && depthJudge(col-1, row) && depthJudge(col, row) && depthJudge(col+1, row))//线段点判断
        {
            tempContour.push_back(Point(col, row));
            circle(dstImage, Point(col, row), 1, Scalar(255, 0, 0), 1, 8);
        }
        else if(lineBegin && depthJudge(col-1, row) && depthJudge(col, row) && !depthJudge(col+1, row))//线段终点判断
        {
            circle(dstImage, Point(col, row), 1, Scalar(0, 0, 255), 1, 8);
            tempContour.push_back(Point(col, row));
            lineBegin = false;
            contours.push_back(tempContour);
            vector<Point>().swap(tempContour);
        }
        else
        {

        }
    }
    // cout << "contours size:" << contours.size() << endl;

    Point center; //定义圆中心坐标

    for(int i = 0; i < contours.size(); i++)
    {
        // cout << "contours " << i+1 << " size:" << contours[i].size() << endl;
        if(contours[i].size() > lowRadiu && contours[i].size() < upRadiu)//筛选掉离散点
        {
            center = Point(contours[i][0].x + contours[i].size() / 2, contours[i][0].y);

            TargetOutPut tempTarget = ColorJudge(center);

            if(tempTarget.color != NONE)
            {
                targetVector.push_back(tempTarget);
                circle(dstImage, center, contours[i].size() / 2, Scalar(255, 0, 0), 2, 8); //绘制第i个轮廓的最小外接圆
                circle(dstImage, center, 1, Scalar(255, 0, 0), 2, 8);
            }
        }
    }


    if(task == BALL)//球矫正最左，桶矫正最右
    {
        sort(targetVector.begin(), targetVector.end(), compTargetLeftToRight);//根据像素点X坐标对目标排序，从左到右
    }
    else if(task == BUCKET)
    {
        sort(targetVector.begin(), targetVector.end(), compTargetRightToLeft);//根据像素点X坐标对目标排序，从右到左
    }

    if(targetVector.size() == 0)
    {
        return false;
    }


    if(targetVector.size() != 0)//矫正,这里需要加条件
    {
        Point2f tempTargetCenter;

        if(firstColor == BLACK && ballOrBucket == BALL)                 //这段是判断如果最左边为黑球的话，把第二个的角度距离给运控，仅限取球
        {
            cout << "now is black and ball" << endl;
            if(targetVector[0].color == secondColor)
            {
                tempTargetCenter.x = targetVector[0].center.x;
                tempTargetCenter.y = targetVector[0].center.y;
            }
            else if(targetVector.size() >= 2 && targetVector[1].color == secondColor)
            {
                tempTargetCenter.x = targetVector[1].center.x;
                tempTargetCenter.y = targetVector[1].center.y;
            }
            else
            {
                cout << "now is no ball" << endl;
                locateFlag = 0;
            }
        }
        else if(firstColor == NONE)
        {
            tempTargetCenter.x = targetVector[0].center.x;
            tempTargetCenter.y = targetVector[0].center.y;
        }
        else
        {
            cout << "now is not black or ball" << endl;
            if(targetVector[0].color == firstColor)
            {
                tempTargetCenter.x = targetVector[0].center.x;
                tempTargetCenter.y = targetVector[0].center.y;
            }
            else
            {
                locateFlag = 0;
            }
        }

        // tempTargetCenter.x = targetVector[0].center.x;
        // tempTargetCenter.y = targetVector[0].center.y;

        int row = (int)tempTargetCenter.y;
        int col = (int)tempTargetCenter.x;
        // depthPixel[0] = invRotationMatix.at<double>(0, 0) * (float)tempTargetCenter.x + invRotationMatix.at<double>(0, 1) * (float)tempTargetCenter.y + invRotationMatix.at<double>(0, 2);
        // depthPixel[1] = invRotationMatix.at<double>(1, 0) * (float)tempTargetCenter.x + invRotationMatix.at<double>(1, 1) * (float)tempTargetCenter.y + invRotationMatix.at<double>(1, 2);

        rotatePixelXY << targetVector[0].center.x, targetVector[0].center.y, 1.f;
        srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
        depthPixel[0] = (int)srcPixelXY(0,0);
        depthPixel[1] = (int)srcPixelXY(0,1);
        // depthPixel[0] = col;
        // depthPixel[1] = row;
        depth = rotateDepthImage.ptr<ushort>(row)[col] / 4000.f;

        // cout << "depth:" << depth << endl;

        // cout << "depthPixel[0]:" << depthPixel[0] << endl;
        // cout << "depthPixel[1]:" << depthPixel[1] << endl;
    
        rs2_deproject_pixel_to_point(realPoint , &color_intrin, depthPixel ,depth);
        if(realPoint[2] < 0.3f)
        {
            cout << "realPointZ error !!!!!!!!!" << endl;
        }
        else
        {
            Vector3d targetPoint = rotateVector * Vector3d(realPoint[0], realPoint[1], realPoint[2]);

            float targetX = targetPoint[0];
            float targetZ = targetPoint[2] + ROBOTCENTER_Z + targetDepth;
            ballBucketAngle = -atan2(targetX, targetZ) / CV_PI * 180.f + angleCompensation;
            ballBucketDis = powf(pow(targetX, 2) + pow(targetZ, 2), 0.5) * 1000.f;
            if(fabs(lastTargetAngle) > 0.1 && fabs(ballBucketAngle - lastTargetAngle) > 10.f)   //后面是保证上个周期没有识别到的情况
            {
                locateFlag = 0;
            }
            else
            {
                locateFlag = 1;
            }

            cout << "targetX:" << targetX << endl;
            cout << "targetZ:" << targetZ << endl;
            cout << "ballBucketAngle:" << ballBucketAngle << endl;
            cout << "ballBucketDis:" << ballBucketDis << endl;
        }
    }
    else
    {
        locateFlag = 0;
    }

    cout << "sideWallCameraX:" << sideWallCameraX << endl;
    cout << "pinkFlag:" << pinkFlag << endl;
    cout << "greenFlag:" << greenFlag << endl;
    cout << "blueFlag:" << blueFlag << endl;
    cout << "blackFlag:" << blackFlag << endl;
    cout << "whiteFlag:" << whiteFlag << endl;
    float Xspace;
    if(targetVector.size() == 5 && pinkFlag && greenFlag && blueFlag && blackFlag && whiteFlag)//判断连续识别
    {
        accCount++;
    }
    else if(targetVector.size() == 4 && pinkFlag && greenFlag && blueFlag && whiteFlag)//只识别除了黑球的4个球的情况
    {
        float Xaverage = (targetVector[0].real3D.x + targetVector[1].real3D.x + targetVector[2].real3D.x + targetVector[3].real3D.x) / 4.f;
        float Xse = (powf((targetVector[0].real3D.x - Xaverage), 2.0) + powf((targetVector[1].real3D.x - Xaverage), 2.0) + powf((targetVector[2].real3D.x - Xaverage), 2.0) + powf((targetVector[3].real3D.x - Xaverage), 2.0)) / 4.f;
        Xaverage = Xaverage - sideWallCameraX / 4.f;

        Xspace = targetVector[3].real3D.x - targetVector[0].real3D.x;
        // cout << "targetVector[3].real3D.x:" << targetVector[3].real3D.x << endl;
        // cout << "targetVector[2].real3D.x:" << targetVector[2].real3D.x << endl;
        // cout << "targetVector[1].real3D.x:" << targetVector[1].real3D.x << endl;
        // cout << "targetVector[0].real3D.x:" << targetVector[0].real3D.x << endl;
        // cout << "Xspace:" << Xspace << endl;
        // cout << "Xspace:" << (Xspace - 4 * targetInterval) << endl;
        // float Xmiddle = (targetVector[3].real3D.x + targetVector[0].real3D.x) / 2;
        TargetOutPut blackBall;
        if(fabs(Xspace - 4 * targetInterval) < 0.150)//黑球不在两边
        {
            for(int i = 1; i < 4; i++)
            {
                float ifHaveTargetItShouldX = sideWallCameraX + firstTargetBeginX + i * targetInterval;//i是场地中的球坑序号，如果这个坑真的有球在侧墙坐标系下的X
                if(fabs(targetVector[i].real3D.x - ifHaveTargetItShouldX) > 0.150)//这个坑没球,，填入黑球
                {
                    accCount++;
                    blackBall.color = BLACK;
                    // blackFlag = true;
                    targetVector.insert(targetVector.begin() + i, blackBall);
                    break;
                }
            }
        }
        else if(fabs(targetVector[0].real3D.x - sideWallCameraX - firstTargetBeginX - 1 * targetInterval) < 0.150)   //目标在近墙端
        {
            cout << "black is left" << endl;
            accCount++;
            blackBall.color = BLACK;
            // blackFlag = true;
            targetVector.insert(targetVector.begin(), blackBall);
        }
        else if(fabs(targetVector[3].real3D.x - sideWallCameraX - firstTargetBeginX - 3 * targetInterval) < 0.150)  //目标在远墙端
        {
            cout << "black is right" << endl;
            accCount++;
            blackBall.color = BLACK;
            // blackFlag = true;
            targetVector.push_back(blackBall);
        }
        else
        {
            accCount = 0;
        }
    }
    else
    {
        accCount = 0;
    }

    if(accCount > 0)//多周期筛选,使用要记得把colorFlag的周期清零取消
    {
        colorFlag = 1;
        rightTime++;
    }
    else
    {
        colorFlag = 0;
    }

    cout << "find targetVector " << targetVector.size() << ": ";
    // Tout << "colorMsg:" << (int)colorFlag << "\t" << Xspace << "\t";

    for(int i = 0; i < targetVector.size(); i++)
    {
        cout << (int)targetVector[i].color << " ";
        // Tout << (int)targetVector[i].real3D.x << "\t";
        colorMsg[i] = targetVector[i].color;
        if(colorFlag == 1)
        {
            firstColor = targetVector[0].color;
            secondColor = targetVector[1].color;
        }
    }
    cout << endl << "firstColor:" << (int)firstColor << endl;
    cout << endl;   
}

bool ActL515::ImageDepthFilter(Mat &inImage, Rect roi, int lowDepth, int upDepth, char maxVal)		//深度滤波
{
    Mat targetDepth = Mat(rotateDepthImage, roi);

    for (int i = 0; i < inImage.rows; i++)
    {        
        for (int j = 0; j < inImage.cols; j++)
        {       
            if((targetDepth.ptr<ushort>(i)[j] > 4 * lowDepth) && (targetDepth.ptr<ushort>(i)[j] < 4 * upDepth))
            {
            }
            else
            {
                if(inImage.channels() == 1)
                {
                    inImage.ptr<uchar>(i)[j] = maxVal;
                }
                else if(inImage.channels() == 3)
                {
                    inImage.ptr<Vec3b>(i)[j][0] = maxVal;
                    inImage.ptr<Vec3b>(i)[j][1] = maxVal;
                    inImage.ptr<Vec3b>(i)[j][2] = maxVal;
                }
            }
        }
    }

    return true;
}

TargetOutPut ActL515::ColorJudge(Point center)										//颜色判断
{
    TargetOutPut temp;

    int y0 = center.y;
    auto HSV = HSVImage.ptr(y0);
    int hChannel = HSV[center.x * 3];
    int sChannel = HSV[center.x * 3 + 1];
    int vChannel = HSV[center.x * 3 + 2];

    auto BGR = rotateImage.ptr(y0);
    int bChannel = BGR[center.x * 3];
    int gChannel = BGR[center.x * 3 + 1];
    int rChannel = BGR[center.x * 3 + 2];

    temp.center.x = center.x;
    temp.center.y = center.y;

    if(!blueFlag && hChannel >= BLUE_MIN_H && hChannel <= BLUE_MAX_H
    && sChannel > BLUE_MIN_S)          //蓝色
    {
        temp.color = BLUE;
        blueFlag = true;
    }
    else if(!greenFlag && hChannel >= GREEN_MIN_H && hChannel <= GREEN_MAX_H
         && sChannel >= GREEN_MIN_S && vChannel >= GREEN_MIN_V) //绿色
    {
        temp.color = GREEN;
        greenFlag = true;
    }
    else if(!pinkFlag && hChannel >= PINK_MIN_H && hChannel <= PINK_MAX_H
         && sChannel >= PINK_MIN_S && sChannel <= PINK_MAX_S)   //粉色
    {
        temp.color = PINK;
        pinkFlag = true;
    }
    else if(!whiteFlag && sChannel <= WHITE_MAX_S && vChannel >= WHITE_MIN_V) //白色
    {
        temp.color = WHITE;
        whiteFlag = true;
    }
    else if(!blackFlag && vChannel <= BLACK_MAX_V)              //黑色
    {
        temp.color = BLACK;
        blackFlag = true;
    }
    else
    {
        temp.color = NONE;
        return temp;
    }

    float realPoint[3];
    float depthPixel[2];
    int col = center.x;
    int row = center.y;
    // depthPixel[0] = invRotationMatix.at<double>(0, 0) * (float)col + invRotationMatix.at<double>(0, 1) * (float)row + invRotationMatix.at<double>(0, 2);
    // depthPixel[1] = invRotationMatix.at<double>(1, 0) * (float)col + invRotationMatix.at<double>(1, 1) * (float)row + invRotationMatix.at<double>(1, 2);
    rotatePixelXY << col, row, 1.f;
    srcPixelXY = rotatePixelXY * imageInvRotateMatrix.transpose();
    depthPixel[0] = round(srcPixelXY(0,0));
    depthPixel[1] = round(srcPixelXY(0,1));
    float depth = depthImage.ptr<ushort>(row)[col] / 4000.f;

    float tempDis = wallDis / 4000.f;

    // if(depth > tempDis && (depth < (tempDis + 0.120)))
    // {
        
    // }
    // else
    // {
    //     temp.color = NONE;
    //     return temp;
    // }

    rs2_deproject_pixel_to_point(realPoint , &color_intrin, depthPixel ,depth);

    Vector3d targetPoint = rotateVector * Vector3d(realPoint[0], realPoint[1], realPoint[2]);

    temp.real3D.x = targetPoint[0];
    temp.real3D.y = targetPoint[1];
    temp.real3D.z = targetPoint[2];

    return temp;
}

bool ActL515::depthJudge(int x, int y)//像素点深度判断
{
    if(rotateDepthImage.ptr<ushort>(y)[x] > wallDis && (rotateDepthImage.ptr<ushort>(y)[x] < (wallDis + 4 * 120)))//目标是墙至墙后150内的点,*4是因为L515深度距离是实际的4倍
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ActL515::showImage(void)
{
	cv::imshow("srcImage", srcImage);
	// cv::imshow("rotateDepthImage", rotateDepthImage);
	// cv::imshow("HSVImage", HSVImage);
    // cv::imshow("rotateImage", rotateImage);
    #ifdef RECORD_VIDEO
    writer << srcImage;
    #endif

    cv::imshow("dstImage", dstImage);
	cv::waitKey(4);
}

void ActL515::recordData(void)
{
    #ifdef TXT
    Fout << timeFlag         << "\t" << (int)filed       << "\t" << (int)ballOrBucket << "\t" << (int)baffleOrLeft << "\t" << robotX           << "\t" << robotRoll << "\t" << robotYaw << "\t" 
         << (int)colorFlag   << "\t" << (int)locateFlag  << "\t" << (int)lBaffleFlag  << "\t" << sideWallCameraX   << "\t" 
         << (int)colorMsg[0] << "\t" << (int)colorMsg[1] << "\t" << (int)colorMsg[2]  << "\t" << (int)colorMsg[3]  << "\t" << (int)colorMsg[4] << "\t" 
         << ballBucketAngle  << "\t" << ballBucketDis    << "\t" << baffleAngle       << "\t" << lBaffleDis        << "\t" << wallYawAngle     << "\t";
    #endif
}

void ActL515::release(void)
{
	srcImage.release();
	depthImage.release();
	HSVImage.release();
	rotateImage.release();
	rotateDepthImage.release();
    colorFlag = 0;
    locateFlag = 0;
    pinkFlag  = false;
    greenFlag = false;
    blueFlag  = false;
    blackFlag = false;
    whiteFlag = false;
    colorMsg[0] = NONE;
    colorMsg[1] = NONE;
    colorMsg[2] = NONE;
    colorMsg[3] = NONE;
    colorMsg[4] = NONE;
    locateFlag = 0;
    lBaffleFlag = 0;
    lastTargetAngle = ballBucketAngle;
    ballBucketAngle = 0.f;//球或桶的角度
    ballBucketDis = 0.f;//球或桶的距离
    baffleAngle = 0.f;//过道中心角度
    lBaffleDis = 0.f;//过道中心或侧墙距离
    wallYawAngle = 0.f;

    // robotRoll = 0.f;//如果通信异常，这几个信息用上个周期的会好一点，除非通信一直异常
    // robotYaw = 0.f;//
    // robotX = 0.f;//

}

void ActL515::getCameraExtrinsics(rs2::frameset alignProcess)
{
	rs2::video_frame colorFrame = alignProcess.get_color_frame();				//获取彩色帧
	rs2::depth_frame alignedDepthFrame = alignProcess.get_depth_frame();

	rs2::stream_profile dprofile = alignedDepthFrame.get_profile();								//读取对齐深度帧参数
    rs2::stream_profile cprofile = colorFrame.get_profile();									//读取对齐颜色帧参数
    
    rs2::video_stream_profile cvsprofile(cprofile);
    color_intrin = cvsprofile.get_intrinsics();													//获取彩色相机内参

    rs2::video_stream_profile dvsprofile(dprofile);
    depth_intrin = dvsprofile.get_intrinsics();													//获取深度相机内参

    depth2color_extrin = dprofile.get_extrinsics_to(cprofile);									//获取深度向彩色相机外参
    color2depth_extrin = cprofile.get_extrinsics_to(dprofile);									//获取彩色向深度相机外参
}

void ActL515::getMCmessage(void)                        //获得运动控制通信数据,发送和接收的顺序都是combine中的顺序
{
    static int recordErrorTime = 0;

    ProcessCommRead(recCvData, MC_CV_BUF_LENGTH);//读取运动控制发来的定位的数据，复制到recData中
    dataUnion roll_Union;
    dataUnion yaw_Union;
    dataUnion X_Union;
    if(recCvData[0] == 'M' && recCvData[1] == 'C' && recCvData[MC_CV_BUF_LENGTH - 2] == '\r' && recCvData[MC_CV_BUF_LENGTH - 1] == '\n')
    {
        cout << "McDataRecognize sucess" << endl;
        // std::lock_guard<std::mutex> lock(communMutex);
        filed = recCvData[2];
        ballOrBucket = recCvData[3];
        baffleOrLeft = recCvData[4];

        CopyData(&recCvData[5],&roll_Union.dataChar[0],4); 
        CopyData(&recCvData[9],&yaw_Union.dataChar[0],4);
        CopyData(&recCvData[13],&X_Union.dataChar[0],4);

        robotRoll = roll_Union.data;
        robotYaw = yaw_Union.data;
        robotX = X_Union.data;
        recordErrorTime = 0;
    } 
    else
    {
        recordErrorTime++;
        if(recordErrorTime > 10)
        {
            cout << "McDataRecognize error" << endl;
        }
    }    
}

char ActL515::SendMessage(void)							//发送数据给运动控制
{
    dataUnion ballBucketAngle_Union;
    dataUnion ballBucketDis_Union;
    dataUnion baffleAngle_Union;
    dataUnion lBaffleDis_Union;
    dataUnion wallYawAngle_Union;
    
    // std::lock_guard<std::mutex> lock(communMutex);
    ballBucketAngle_Union.data = ballBucketAngle;
    ballBucketDis_Union.data = ballBucketDis;
    baffleAngle_Union.data = baffleAngle;
    lBaffleDis_Union.data = lBaffleDis;
    wallYawAngle_Union.data = wallYawAngle;
    cvData[0] = 'C';
    cvData[1] = 'V';
    cvData[2] = timeFlag;
    cvData[3] = colorFlag;
    cvData[4] = locateFlag;
    cvData[5] = lBaffleFlag;
    cvData[6] = colorMsg[0];
    cvData[7] = colorMsg[1];
    cvData[8] = colorMsg[2];
    cvData[9] = colorMsg[3];
    cvData[10] = colorMsg[4];
    CopyData(&ballBucketAngle_Union.dataChar[0],&cvData[11],4); 
    CopyData(&ballBucketDis_Union.dataChar[0],&cvData[15],4);
    CopyData(&baffleAngle_Union.dataChar[0],&cvData[19],4);
    CopyData(&lBaffleDis_Union.dataChar[0],&cvData[23],4);
    CopyData(&wallYawAngle_Union.dataChar[0],&cvData[27],4);

    cvData[31] = '\r';
    cvData[32] = '\n';
    ProcessCommWrite(cvData, MC_CV_BUF_LENGTH);
}

ActL515::~ActL515()
{    
	delete[] data;
}


#define GYRO_INIT_PERIOD 10                           //
inline void processGyro(rs2_vector gyro_data, double ts)
{
    static float3 sumGyro = {0.f, 0.f, 0.f};
    static bool initOk = false;

	if(!initOk)
    {
        if (initGyro < GYRO_INIT_PERIOD) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            last_ts_gyro = ts;
            // initialTheta.x = gyro_data.x;
            // initialTheta.y = gyro_data.y;
            // initialTheta.z = gyro_data.z;
            sumGyro.add(gyro_data.x, gyro_data.y, gyro_data.z);
            initGyro++;
            initOk = false;
            return;
        }
        else
        {
            cout << "gyro init ok !!!" << endl;
            if(!initOk)
            {
                initOk = true;
                std::lock_guard<std::mutex> lock(thetaMtx);
                initialTheta.x = sumGyro.x / float(GYRO_INIT_PERIOD);
                initialTheta.y = sumGyro.y / float(GYRO_INIT_PERIOD);
                initialTheta.z = sumGyro.z / float(GYRO_INIT_PERIOD);
            }
        }
    }
    // else
    // {
    //     cout << "gyro init ok !!!" << endl;
    // }

    // if (initGyro >= 0) // On the first iteration, use only data from accelerometer to set the camera's initial position
	// {
    //     std::lock_guard<std::mutex> lock(thetaMtx);
	// 	initGyro--;
	// 	last_ts_gyro = ts;
	// 	initialTheta.x = gyro_data.x;
	// 	initialTheta.y = gyro_data.y;
	// 	initialTheta.z = gyro_data.z;
	// 	return;
	// }
	// Holds the change in angle, as calculated from gyro
	float3 gyro_angle;

	// Initialize gyro_angle with data from gyro
	gyro_angle.x = gyro_data.x; // Pitch
	gyro_angle.y = gyro_data.y; // Yaw
	gyro_angle.z = gyro_data.z; // Roll

	// Compute the difference between arrival times of previous and current gyro frames
	double dt_gyro = (ts - last_ts_gyro) / 1000.0;
	last_ts_gyro = ts;

	// Change in angle equals gyro measures * time passed since last measurement
	gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

	// Apply the calculated change of angle to the current angle (theta)
	std::lock_guard<std::mutex> lock(thetaMtx);
	theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    thetaCameraChange = (theta - initialTheta);
    thetaCameraChange.toDegree();

    // Tout << "theta:" << "\t" << theta.x << "\t" << theta.y << "\t" << theta.z;
    // Tout << "\t" << "rollCameraChange:" << "\t" << rollCameraChange << endl;

}

inline void processAccel(rs2_vector accel_data)
{
	// Holds the angle as calculated from accelerometer data
	float3 accel_angle;

	// Calculate rotation angle from accelerometer data
	accel_angle.z = atan2(accel_data.y, accel_data.z);
	accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

	// If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
	std::lock_guard<std::mutex> lock(thetaMtx);
	if (firstAccel >= 0)
	{
		firstAccel--;
		theta = accel_angle;
		// Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
		theta.y = CV_PI;
	}
	else
	{
		/* 
		Apply Complementary Filter:
			- high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
				that are steady over time, is used to cancel out drift.
			- low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
		*/
		theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
		theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
	}

    thetaCameraChange = (theta - initialTheta);
    thetaCameraChange.toDegree();
    // Tout << "theta:" << "\t" << theta.x << "\t" << theta.y << "\t" << theta.z;
    // Tout << "\t" << "rollCameraChange:" << "\t" << rollCameraChange << endl;
}

bool check_imu_is_supported()
{
    bool found_gyro = false;
    bool found_accel = false;
    rs2::context ctx;
    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

bool compContours(const vector<Point> &a, const vector<Point> &b)
{
    return contourArea(a) > contourArea(b);
}

bool compTargetLeftToRight(const TargetOutPut &a, const TargetOutPut &b)//从左到右排序
{
    return a.center.x < b.center.x;
}

bool compTargetRightToLeft(const TargetOutPut &a, const TargetOutPut &b)//从右到左排序
{
    return a.center.x > b.center.x;
}

void CopyData(char* origen, char* afterTreat, int size)
{
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}