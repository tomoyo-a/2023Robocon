#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include "data.h"
#include "pos.h"
#include "pps.h"

extern vector<LineParam_t> wallLine;
extern Mat radarImage;
extern Robot_t robot;
extern ofstream outFile;
vector<LineParam_t> allCalculateLine;
vector<int> cornerPos;
const int IN_ONE_LINE = 30;
extern atomic_int proStamp;
//jiesuanjiaodusuo
extern pthread_mutex_t mtxCorrect;
extern ofstream output;
using namespace std;

//清空数据
void DataClear(void)
{
	robot.radarDistance.clear();
	robot.radarTheta.clear();
	robot.radarPoint.clear();
	allCalculateLine.clear();
	cornerPos.clear();
}

//数据处理,找到所有的角点并保存
void DataProcess(void)
{
	int dataLeath = robot.radarPoint.size();
	SaveCornerPos(0, dataLeath - 1);
	cornerPos.push_back(0);
	cornerPos.push_back(dataLeath - 1);
}

//找到角点
int FindCornerPos(int start, int end)
{
	LineParam_t singleLineParam;
	float pToLineDistance = 0;
	float maxDistance = -1;
	int featurePos = 0;
	if (robot.radarPoint.at(start).x == robot.radarPoint.at(end).x)
	{
		singleLineParam.A = 1.f;
		singleLineParam.C = (float)-(robot.radarPoint.at(start).x + robot.radarPoint.at(end).x) / 2;
		singleLineParam.B = 0;
	}
	if (robot.radarPoint.at(start).y == robot.radarPoint.at(end).y)
	{
		singleLineParam.A = 0;
		singleLineParam.C = (float)-(robot.radarPoint.at(start).y + robot.radarPoint.at(end).y) / 2;
		singleLineParam.B = 1.;
	}
	if (robot.radarPoint.at(start).x != robot.radarPoint.at(end).x && robot.radarPoint.at(start).y != robot.radarPoint.at(end).y)
	{
		singleLineParam.A = 1;
		singleLineParam.B = -(robot.radarPoint.at(end).x - robot.radarPoint.at(start).x) / (float)(robot.radarPoint.at(end).y - robot.radarPoint.at(start).y);
		singleLineParam.C = (robot.radarPoint.at(end).x - robot.radarPoint.at(start).x) / (float)(robot.radarPoint.at(end).y - robot.radarPoint.at(start).y) * robot.radarPoint.at(start).y - robot.radarPoint.at(start).x;
	}

	for (int i = start; i < end; i++)
	{
		pToLineDistance = fabs(singleLineParam.A * robot.radarPoint.at(i).x + singleLineParam.B * robot.radarPoint.at(i).y + singleLineParam.C) /
						  sqrt(pow(singleLineParam.A, 2) + pow(singleLineParam.B, 2));
		if (pToLineDistance > maxDistance)
		{
			maxDistance = pToLineDistance;
			featurePos = i;
		}
	}
	if (maxDistance > 50)
	{
		//输出起点和终点以及角点位置和坐标
		//cout << start << "\t" << end << "\t" << featurePos << "\tP\t" << robot.radarPoint[featurePos].x
		//	 << "\t" <<robot.radarPoint[featurePos].y << "\t" << maxDistance << endl;
		return featurePos;
	}
	else
	{
		return 0;
	}
}

//保存该次数据中的角点
void SaveCornerPos(int start, int end)
{
	int startCornerPos, endCornerPos, midCornerPos;
	startCornerPos = start;
	endCornerPos = end;
	midCornerPos = FindCornerPos(startCornerPos, endCornerPos);
	if (midCornerPos == 0)
	{
		return;
	}
	else
	{
		cornerPos.push_back(midCornerPos);
		SaveCornerPos(start, midCornerPos);
		SaveCornerPos(midCornerPos, end);
	}
}

//计算直线参数
void GetLine(void)
{
	cv::Point lineBegin,lineEnd,tmpPoint;
	LineParam_t tmpLine;
	float p2Line = 0;
	cout<<"cnum: "<<cornerPos.size()<<endl;
	sort(cornerPos.begin(), cornerPos.end());
	//cout  << "CO ";
	// for (int i = 0; i < cornerPos.size(); i++)
	// {
	// 	cout << cornerPos.at(i) << ' ';
	// }
	cout << "Point NUM: " << robot.radarPoint.size() << endl;
	for (int i = 0; i < cornerPos.size() - 1; i++)
	{
		// cout<<"OLES:"<<i<<" "<<cornerPos.at(i)<<" "<<cornerPos.at(i + 1)<<endl;
		tmpLine = LineFitLeastSquares(robot.radarPoint, cornerPos.at(i), cornerPos.at(i + 1));
		// cout << "L " << tmpLine << endl;
		bool hbCorr = false;
		//如果直线上点数太少 规定tmpLine.A=9999;，不拟合
		//点数足够
		if (tmpLine.angle < 360)
		{
			lineBegin = robot.radarPoint[cornerPos.at(i)];
			//该判断截止到倒数第三个和倒数第二个点的拟合
			if (i < cornerPos.size() - 2)
			{
				//cout<<cornerPos.size()<<' '<<tmpLine.A<<' '<<tmpLine.B<<endl;
				//如果下一个角点距上一条拟合的直线较近，则认为在同一条直线上
				p2Line = P2LineDistance(tmpLine, robot.radarPoint[cornerPos.at(i + 2)]);
				// cout << "P2L:" << p2Line << "\t" << endl;
				if (p2Line < IN_ONE_LINE && (cornerPos.at(i + 2) - cornerPos.at(i + 1)) > 15)
				{
					// cout<<"OLES:"<<i<<" "<<cornerPos.at(i)<<" "<<cornerPos.at(i + 2)<<endl;
					tmpLine = LineFitLeastSquares(robot.radarPoint, cornerPos.at(i), cornerPos.at(i + 2));
					// cout << "合并角点：" << (i + 1) << "\t" << (i + 2) << "\t" << endl;
					i++;
					hbCorr = true;
					lineEnd = robot.radarPoint[cornerPos.at(i + 2)];
				}
			}
			allCalculateLine.push_back(tmpLine);
		}
	}
	MergeLine(allCalculateLine);
}
extern ofstream outangle;
//最小二乘法拟合直线
LineParam_t LineFitLeastSquares(vector<cv::Point> radarPoint, int start, int end)
{
	LineParam_t singleLineParam;
	float sumXX = 0.0;
	float sumYY = 0.0;
	float sumX = 0.0;
	float sumXY = 0.0;
	float sumY = 0.0;
	int dataLenth = end - start;
	float k1 = 0.4, k2 = 0.6;
	bool rotateFlag = 0;
	//d1 直线起点处两点间距,d2 直线中点处两点间距，lineLenth 直线总的长度
	int d1 = 0, d2 = 0, lineLenth = 0;
	Point p1, p2;
	p1 = radarPoint.at(start), p2 = radarPoint.at(end);
	lineLenth = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
	float featureNum = k1 * dataLenth + k2 * lineLenth / 50;
	// cout <<" feature: "<< start<<' '<<end<<' '<<lineLenth << ' ' << dataLenth << ' ' << featureNum << endl;

	//如果点数较多或直线比较长
	if (featureNum > 30 && dataLenth > 30)
	{
		//直线前后各缩几个点 防止角点没有精准识别
		start = start + 5;
		end = end - 5;
		dataLenth = end - start;
		// cv::circle(radarImage, P2Img(radarPoint.at(start)), 5, cv::Scalar(0, 255, 255));
	}
	//如果点数较少或直线较短 不拟合
	else
	{
		singleLineParam.angle = 9999;
		// output << "pointnum too small!" << endl;
		return singleLineParam;
	}
	//一共循环计算了start-end个数据
	for (int i = start; i < end; i++)
	{
		sumXX += radarPoint.at(i).x * radarPoint.at(i).x;
		sumX += radarPoint.at(i).x;
		sumXY += radarPoint.at(i).x * radarPoint.at(i).y;
		sumY += radarPoint.at(i).y;
	}

	//计算均值
	float averageX, averageY;
	averageX = sumX / dataLenth;
	averageY = sumY / dataLenth;

	// 计算斜率a和截距b
	//y=ax+b
	float a, b, denominator = 0;
	denominator = dataLenth * sumXX - sumX * sumX;
	if (denominator == 0)
	{
		singleLineParam.angle = 9999;
		cout << "denominator is zero!!!" << endl;
		return singleLineParam;
	}
	else
	{
		a = (dataLenth * sumXY - sumX * sumY) / denominator;
		b = averageY - a * averageX;
	}

	/*R 拟合优度 R越接近1，拟合度越好
		MSE 均方误差
		varY Y的方差
		https://www.cnblogs.com/aviator999/p/10049646.html
	*/
	float R2 = 0, MSE = 0, varY = 0;
	float sumErr = 0, tmpErr = 0;
	for (int i = start; i < end; i++)
	{
		tmpErr = radarPoint.at(i).y - (a * radarPoint.at(i).x + b);
		MSE += tmpErr * tmpErr;
		tmpErr = radarPoint.at(i).y - averageY;
		varY += tmpErr * tmpErr;
	}
	R2 = 1 - MSE / varY;

	float tmpAng = 0;
	tmpAng = RADTOANG(atan(a));
	singleLineParam.angle = tmpAng;
	// outFile << "L " << tmpAng << ' '<<a<<' '<<b<<' ';
	//90度直线交换XY重新拟合
	if (fabs(fabs(tmpAng) - 90) < 30)
	{
		rotateFlag = 1;
		// outangle << "Fit " << start << " " << end;
		// outangle << " first " << tmpAng << " " << a << " " << b;
		// outangle<< " data1 "<<dataLenth << ' '<<sumXX<<' '<<sumXY<< ' ' <<sumX<<' ' <<sumY;
		for (int i = start; i < end; i++)
		{
			//把y坐标当作x坐标 x坐标当作y
			sumYY += radarPoint.at(i).y * radarPoint.at(i).y;
		}
		// outangle<< " data2 "<<dataLenth << ' '<<sumYY<<' '<<sumXY<< ' ' <<sumX<<' ' <<sumY;
		denominator = dataLenth * sumYY - sumY * sumY;
		if(denominator !=0)
		{
			a = (dataLenth * sumXY - sumX * sumY) / denominator;
			b = averageX - a * averageY;
			// outangle << " mid " << a << " " << b;
			b = -b / a;
			a = 1 / a;
			singleLineParam.angle = RADTOANG(atan(a));
		}
		// outangle << " last " << singleLineParam.angle << " " << a << " " << b << endl;
	}
	// outFile << rotateFlag << ' ' << singleLineParam.angle <<' '<<a<<' '<<b<< endl;

	// outangle<< "Fit " << rotateFlag<<' '<< start << " " << end;
	// outangle<< " data "<<dataLenth << ' '<<sumXX<<' '<<sumXY<< ' ' <<sumX<<' ' <<sumY
	// 		<< ' '<<a<<' '<<b<<' '<<singleLineParam.angle<<endl;
	//补全直线参数
	singleLineParam.A = -a;
	singleLineParam.B = 1;
	singleLineParam.C = -b;

	singleLineParam.pNum = end - start;
	singleLineParam.startP = start; //拟合时补充直线参数，角度在EC函数里面已经计算好
	singleLineParam.endP = end;
	singleLineParam.startPoint = radarPoint.at(start);
	singleLineParam.endPoint = radarPoint.at(end);
	singleLineParam.slope = a;
	singleLineParam.intercept = b;
	cout << "flag_L" << endl;
	// cout<<"290: "<<singleLineParam.startP<<' '<<singleLineParam.endP<<' '<<endl;
	return singleLineParam;
}

//对容器内的角点进行排序
void Sort(void)
{
	int dataLenth = cornerPos.size();
	int exchangeNum = 0;
	for (int i = 0; i < dataLenth - 1; i++)
	{
		for (int j = i + 1; j < dataLenth; j++)
		{
			if (cornerPos.at(i) > cornerPos.at(j))
			{
				exchangeNum = cornerPos.at(j);
				cornerPos.at(j) = cornerPos.at(i);
				cornerPos.at(i) = exchangeNum;
			}
		}
	}
}

//对拟合的直线进行排序，长的排在前面
void SortLine()
{
	LineParam_t tempLine = {0};
	int numLine = allCalculateLine.size();
	int exchangeNum = 0;
	for (int i = 0; i < numLine - 1; i++)
	{
		for (int j = i + 1; j < numLine; j++)
		{
			if (allCalculateLine.at(i).pNum < allCalculateLine.at(j).pNum)
			{
				tempLine = allCalculateLine.at(i);
				allCalculateLine.at(i) = allCalculateLine.at(j);
				allCalculateLine.at(j) = tempLine;
			}
		}
	}
}

//计算点到直线距离
float P2LineDistance(LineParam_t line, cv::Point point)
{
	float ret = fabs(line.A * point.x + line.B * point.y + line.C) / sqrt(line.A * line.A + line.B * line.B);
	return ret;
}
/*对雷达的数据进行动态补偿
 *@param posData 收到的最新的六个周期定位系统数据
 *@param compenData 计算所得的雷达各个点对应的基于世界坐标系的补偿数据
 */
void DataCompensation(vector<Pos_t> posData, vector<Pos_t> &compenData)
{
	cout << "posData.size " << posData.size() << endl;
	compenData.clear();
	Pos_t deltPosData;
	Pos_t tmpPos;
	if (posData.size() < 5)
	{
		Delay_ms(20);
	}
	//25ms,定位系统5ms，分为五个周期
	for (int i = 0; i < 5; i++)
	{
		//使用矫正后的坐标计算车体当前位置
#ifndef TEST_LIDAR
		tmpPos = posData.at(i);
#else
		tmpPos = posData.at(i);
#endif
		deltPosData.x = (posData.at(i + 1).x - posData.at(i).x) / (POINTNUM / 5);
		deltPosData.y = (posData.at(i + 1).y - posData.at(i).y) / (POINTNUM / 5);
		deltPosData.theta = (posData.at(i + 1).theta - posData.at(i).theta) / (POINTNUM / 5);
		//将角度变化量换算到-180~180之间
		LimitAng(deltPosData.theta);
		int start = 0;
		int end = POINTNUM / 5;
		if (i == 0)
		{
			start = 180;
		}
		else if (i == 4)
		{
			end = (POINTNUM / 5) - 180;
		}
		for (int j = start; j < end; j++)
		{
			Pos_t rayPos((tmpPos.x + deltPosData.x * j), (tmpPos.y + deltPosData.y * j), (tmpPos.theta + deltPosData.theta * j));
			//cout<<"第二个"<< j <<endl;
			compenData.push_back(rayPos);
		}
	}
#ifndef TEST_LIDAR
	tmpPos = posData.at(5);
#else
	tmpPos = posData.at(5);
#endif
	compenData.push_back(tmpPos);
}

/*对已知数据进行坐标变换
 *@param oriPoint 点在原坐标系中的坐标
 *@param xTransform 原坐标系相对于目标坐标系X轴上的平移
 *@param yTransform 原坐标系相对于目标坐标系Y轴上的平移
 *@param radTransform 原坐标系相对于目标坐标系逆时针旋转的角度
 */
cv::Point DataTransform(cv::Point oriPoint, int xTransform, int yTransform, float radTransform)
{
	cv::Point ret;
	ret.x = oriPoint.x * cos(radTransform) - oriPoint.y * sin(radTransform) + xTransform;
	ret.y = oriPoint.y * cos(radTransform) + oriPoint.x * sin(radTransform) + yTransform;
	return ret;
}

/*将角度限制在180~-180之间
 *@param ang: 传入的角度值(单位是度)
 */
float LimitAng(float &ang)
{
	while (ang > 180)
	{
		ang -= 360;
	}
	while (ang < -180)
	{
		ang += 360;
	}
	return ang;
}

//合并直线
void MergeLine(vector<LineParam_t> &allCalculateLine)
{
	if (allCalculateLine.size() > 1)
	{
		//比较直线的角度差值和距离来判断是否是一条直线
		for (int i = 0; i < allCalculateLine.size() - 1; i++)
		{
			// cout<<"**";
			for (int j = i + 1; j < allCalculateLine.size(); j++)
			{
				// cout << "compare " << i << " " << j;
				if (JudgeSameLine(allCalculateLine.at(i), allCalculateLine.at(j)))
				{
					LineParam_t mergeLine;
					//判断直线的角点先后顺序
					if (allCalculateLine.at(i).startP < allCalculateLine.at(j).startP)
					{
						mergeLine = allCalculateLine.at(i) + allCalculateLine.at(j);
						// mergeLine = LineFitLeastSquares(robot.radarPoint, mergeLine.startP, mergeLine.endP);
					}
					else
					{
						mergeLine = allCalculateLine.at(j) + allCalculateLine.at(i);
					}
					//第一条直线和最后一条直线合并时
					if (i == 0 && j == (allCalculateLine.size() - 1))
					{
						//最后一条直线的起点 作为
						mergeLine.startP = allCalculateLine.at(j).startP;
						mergeLine.endP = allCalculateLine.at(i).endP;
						mergeLine.startPoint = allCalculateLine.at(j).startPoint;
						mergeLine.endPoint = allCalculateLine.at(i).endPoint;
					}
					allCalculateLine.at(j) = mergeLine;
					allCalculateLine.erase(allCalculateLine.begin() + i);
					i = i - 1;
					break;
				}
			}
		}
	}
}

/*判断两条直线是否为一条直线
 *@param line1 直线一
 *@param line2 直线二
 *@return 如果是同一条直线返回true
 */
bool JudgeSameLine(LineParam_t line1, LineParam_t line2)
{
	float angleErr = fabs(fabs(line1.angle) - fabs(line2.angle));
	if (angleErr < ANGERRMAX)
	{
		float distan = (P2LineDistance(line1, line2.startPoint) + P2LineDistance(line1, line2.endPoint)) / 2;
		Point p1 = line2.startPoint;
		Point p2 = line2.endPoint;
		float lineDis = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
		distan = distan / lineDis;
		// cout << distan << endl;
		if (distan < DISERR)
		{
			return 1;
		}
		else
		{
			// cout << "distan too big: " << distan << endl;
			return 0;
		}
	}
	else
	{
		// cout << "angleErr too big: " << angleErr << endl;
		return 0;
	}
}
/*输出数据
 *@param logData 输出流
 */
void Robot_t::OutputInformation(ofstream &logData)
{
	auto totalTime = chrono::duration_cast<chrono::microseconds>(timeEnd - timeStart);
	logData << "T " << (int)serialFlag << ' ' << proStamp << ' ' << totalTime.count() / 1000.f << ' ' << robot.drStatus
			<< " P " << ppsRawPos << ' ' << rbotVel << ' ' << ppsErr
			<< " C " << rbotPos << ' ' << ppsCorrect << ' ' << singleCorrect << endl;

	for (int i = 0; i < allCalculateLine.size(); i++)
	{
		logData << "L " << allCalculateLine.at(i) << endl;
	}
	// cout << "T " << proStamp << ' ' << totalTime.count() / 1000.f << ' ' << robot.drStatus
	// 	 << " P " << recentPosData.at(0) << ' ' << ppsErr
	// 	 << " C " << rbotPos << ' ' << ppsCorrect << ' ' << singleCorrect << endl;

	// for (int i = 0; i < allCalculateLine.size(); i++)
	// {
	// 	cout << "L " << allCalculateLine.at(i) << endl;
	// }

	// cout << cornerPos.at(i) << ' ';
	// cout << endl;
	ppsErr = Pos_t();
	singleCorrect = Pos_t();
}

// 存入看到的直线的边界值，起点均为场地直线的端点
vector<LineParam_t> divisionLine;
// 按顺序传入点来计算两点的直线(p1为起点，p2为终点) 以逆时针方向传入点
LineParam_t TwoPFittLine(Point p1, Pos_t p2)
{
	LineParam_t retLine;
	retLine.A = p1.y - (int)p2.y;
	retLine.B = (int)p2.x - p1.x;
	retLine.C = p1.x * (int)p2.y - (int)p2.x * p1.y;
	retLine.startPoint = p1;
	retLine.endPoint = Point((int)p2.x, (int)p2.y);
	if ((int)retLine.B == 0)
	{
		retLine.angle = 90;
	}
	else
	{
		retLine.angle = atan(-retLine.A / retLine.B) * 180 / CV_PI;
		if (retLine.angle < 0)
		{
			retLine.angle += 180;
		}
	}

	return retLine;
}

// 判断直线是否相交，如果相交，返回值为交点，不相交，返回值为原点
Pos_t TwoLineIfIntersect(LineParam_t line1, LineParam_t line2)
{
	Pos_t retPoint;
	// 直接对两直线进行节方程组解出焦点坐标的大小，在判断是否同时分别夹在两直线的端点中间
	float A1 = line1.A, B1 = line1.B, C1 = line1.C;
	float A2 = line2.A, B2 = line2.B, C2 = line2.C;
	Pos_t origin;
	origin.theta = -1;
	int X = 0;
	int Y = 0;
	float denominator = A1 * B2 - A2 * B1;
	if (denominator == 0)
	{
		return origin; // 两直线平行，不相交
	}
	// 两线段不平行才可以求交点
	else
	{
		X = (int)((B1 * C2 - B2 * C1) / denominator);
		Y = (int)((A2 * C1 - A1 * C2) / denominator);
		int D1 = (X - line1.startPoint.x) * (line1.endPoint.x - X);
		int D2 = (Y - line1.startPoint.y) * (line1.endPoint.y - Y);
		int D3 = (X - line2.startPoint.x) * (line2.endPoint.x - X);
		int D4 = (Y - line2.startPoint.y) * (line2.endPoint.y - Y);
		if (D1 == 0 && D2 == 0 && D2 == 0 && D2 == 0) // 两直线交点在端点上面
		{
			return origin; // 两线段不相交
		}
		if (D1 < 0 || D2 < 0 || D3 < 0 || D4 < 0) // 有一个值小于0都认为交点在直线外
		{
			return origin; // 两线段不相交
		}
		// 剩下的情况都是交点在直线中间，包括角度位为0或者90的情况
		else
		{
			retPoint.x = X;
			retPoint.y = Y;
			return retPoint; // 两线段相交，返回值为交点
		}
	}
}

// 场地直线
extern vector<LineParam_t> wallLine;
// 找到挡住直线中离机器人最近的那一条直线
LineParam_t FindIntersectLine(LineParam_t line)
{
	pthread_mutex_lock(&mtxCorrect);
	Pos_t robotPos = robot.recentPosData.at(0);
	;
	pthread_mutex_unlock(&mtxCorrect);
	vector<Pos_t> intersection;
	Point origin(0, 0);
	for (int i = 0; i < 10; i++)
	{
		Pos_t retPoint = TwoLineIfIntersect(line, wallLine.at(i));
		// 如果两线段有交点（即返回值不是（0,0,-1）点），就记录下直线的标号以及交点
		if (retPoint.x != 0 && retPoint.y != 0 && (int)retPoint.theta != -1)
		{
			Pos_t pushPoint; //theta先用来记录被挡住的直线的标号
			pushPoint.x = retPoint.x;
			pushPoint.y = retPoint.y;
			pushPoint.theta = i;
			intersection.push_back(pushPoint);
		}
	}
	// 如果有相交的线，选择离机器人坐标最近的那一个交点的直线
	if (intersection.size())
	{
		// 输出相交直线的标号
		Pos_t temp;
		float distance = 99999.9;
		float tempDistance = 0.0;
		for (int i = 0; i < intersection.size(); i++)
		{
			Point p1;
			p1.x = (int)(robotPos.x);
			p1.y = (int)(robotPos.y);
			Point p2;
			p2.x = (int)intersection.at(i).x;
			p2.y = (int)intersection.at(i).y;
			// 机器人与直线交点的距离
			tempDistance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
			if (tempDistance < distance)
			{
				temp = intersection.at(i);
				distance = tempDistance;
			}
		}
		LineParam_t retLine;
		retLine = wallLine.at((int)temp.theta);
		retLine.wallNum = (int)temp.theta;
		return retLine;
	}
	// 如果没有找到相交的直线，就返回原直线，并且把原直线的wallnum数值赋为-2
	else
	{
		line.wallNum = -1;
		return line;
	}
}

// 按照直线是否被遮挡的方法来分开上的点，并且转换成雷达扫描到的点的标号，记录该方向下看到的直线标号
// 每两条直线中间夹着一条可以看到的场地直线
void DivisionImage(vector<int> &keyPoint, vector<int> &wallNum)
{
	// 运行前清空容器数据
	keyPoint.clear();
	wallNum.clear();
	// cout << "get robotPos data" << endl;
	pthread_mutex_lock(&mtxCorrect);
	Pos_t robotPos = robot.recentPosData.at(0);
	;
	pthread_mutex_unlock(&mtxCorrect);
	cout << "robotPos: " << robotPos << endl;
	// 在外场才要判断看到的是那些直线
	if (robot.drStatus)
	{
		for (int i = 0; i < 10; i++)
		{
			LineParam_t line = TwoPFittLine(wallLine.at(i).startPoint, robotPos);
			LineParam_t retLine = FindIntersectLine(line);
			// 假如没有相交直线，那么这条直线就是视野的边界，这里的直线的起点都是场地直线上的点
			// 然后开始找下一条直线的起点，如果返回的直线的wallNum为-1，那么这条直线就没有被遮挡
			// 如果有相交直线，那返回的是最近的那一条直线并且wallNum是对应的场地直线的标号
			if (retLine.wallNum == -1)
			{
				divisionLine.push_back(retLine);
			}
			// 假如这条直线被挡住了，并且挡住的直线的标号排在就从挡住直线的起点继续运算
			else
			{
				// 假如挡住的直线标号在i前面，就不计算，防止死循环
				if (retLine.wallNum < i)
				{
					i = i;
				}
				else
				{
					i = retLine.wallNum - 1; // i会在最后++，所以要减1
				}
			}
		}
	}
	// 内场就直接连接各直线的端点（如果不用圆台进行矫正）
	else
	{
		for (int i = 10; i < wallLine.size(); i++)
		{
			LineParam_t line = TwoPFittLine(wallLine.at(i).startPoint, robotPos);
			divisionLine.push_back(line);
		}
	}
	// 开始用分割后的直线转换成点的标号，并且确认这些标号中间夹着的直线标号
	if (divisionLine.size())
	{
		// 转换角度在0～360下
		for (int i = 0; i < divisionLine.size(); i++)
		{
			if (divisionLine.at(i).startPoint.y < robotPos.y + robot.ppsCorrect.y)
			{
				divisionLine.at(i).angle = divisionLine.at(i).angle + 180;
			}
			// 把角度旋转到雷达坐标系下
			divisionLine.at(i).angle = divisionLine.at(i).angle - robotPos.theta - robot.ppsInitAngle + 90;
			if (divisionLine.at(i).angle < 0)
			{
				divisionLine.at(i).angle += 360;
			}
		}
		// 筛选特征值和特征值夹着的直线
		for (int i = 0; i < divisionLine.size(); i++)
		{
			// 直线的起点（在场地上的点），用来判断点的标号夹着的直线
			Point point = divisionLine.at(i).startPoint;
			int end = 0;
			int start = 0;
			// 内场
			if (robot.drStatus)
			{
				start = 0;
				end = 10;
			}
			else
			{
				start = 10;
				end = 16;
			}
			// 开始寻找特征值夹着的直线
			// 当找到的直线的起点y值大于机器人坐标时
			if (point.y < (robotPos.y + robot.ppsCorrect.y))
			{
				for (int j = start; j < end; j++)
				{
					// 假如直线的起点与场地直线的起点相同就认为这条直线是可以看到的
					if (wallLine.at(j).startPoint == point)
					{
						wallNum.push_back(j);
						break;
					}
				}
			}
			// 当找到的直线的起点y值小于机器人坐标时
			else
			{
				// 此时就用场地直线的终点来判断是否可以看到直线
				for (int j = start; j < end; j++)
				{
					// 假如直线的起点与场地直线的终点相同就认为这条直线是可以看到的
					if (wallLine.at(j).endPoint == point)
					{
						// j加一，防止推入的是上一条直线
						wallNum.push_back(j + 1);
						break;
					}
				}
			}
			// 计算直线的边界值标号
			if (divisionLine.at(i).angle < 90)
			{
				keyPoint.push_back((int)((45 - divisionLine.at(i).angle) * 4));
			}
			else
			{
				keyPoint.push_back((int)((405 - divisionLine.at(i).angle) * 4));
			}
		}
		keyPoint.push_back(keyPoint.at(0));
	}
	else
	{
		// 没有找到直线，程序错误
		cout << "NO FIND DIVISION LINE" << endl;
	}
	// 如果对s点分好段，就开始判断这些点是否在墙的附近
}

extern vector<LineParam_t> insideZone;
// 筛选雷三达扫描到的点，传入要拟合的直线容器，放入进行拟合的点和对应的场地直线标号
void ScreenRadarPoint()
{
	chrono::steady_clock::time_point timeS = chrono::steady_clock::now();
	cout << "flag1";
	// 和allCalculateLine容器作用相同，测试程序合理性
	vector<LineParam_t> allMatchLine;
	vector<int> keyPoint;	  // 角点标号
	vector<int> wallNum;	  // 角点之间夹着的直线
	vector<int> goodPointKey; // 正确的点的交点标志位
	divisionLine.clear();	  // 清空容器
	DivisionImage(keyPoint, wallNum);
	cout << "flag2";
	// cout << "ScreenRadarPoint size: " << keyPoint.size() << ' ' << wallNum.size() << endl;
	if (keyPoint.size() && wallNum.size())
	{
		// cout << "keyPoint: "; // 输出keypoint的内容
		// for (int i = 0; i < keyPoint.size(); i++)
		// {
		// 	cout << keyPoint.at(i) << " ";
		// }
		// cout << endl;
		// cout << "visionwallNum: "; // 输出可以看到的直线的标号
		// for (int i = 0; i < wallNum.size(); i++)
		// {
		// 	cout << wallNum.at(i) << " ";
		// }
		// cout << endl;
		// 记录机器人在这个点时可以看到的直线，记录好墙号
		for (int i = 0; i < wallNum.size(); i++)
		{

			LineParam_t pushLine;
			pushLine.wallNum = wallNum.at(i);
			allMatchLine.push_back(pushLine);
		}
		const int DATAMINNUM = 30;
		const int LIMITNUM = 5;
		// 存入直线对应的点（在后面滤掉不合理的点）
		for (int i = 0; i < allMatchLine.size(); i++)
		{
			// 点的标号前大后小，代表没有跨越盲区
			if (keyPoint.at(i) > keyPoint.at(i + 1))
			{
				// 标号正常，没有在盲区里面的点
				if (keyPoint.at(i + 1) > 0 && keyPoint.at(i) < 1081 && fabs(keyPoint.at(i) - keyPoint.at(i + 1)) > DATAMINNUM)
				{
					allMatchLine.at(i).linePoint.assign(robot.radarPoint.begin() + keyPoint.at(i + 1) + LIMITNUM, robot.radarPoint.begin() + keyPoint.at(i) - LIMITNUM);
				}
				// 数据部分在-180～0的区域内
				else if (keyPoint.at(i + 1) < 0 && keyPoint.at(i) > 0 && (keyPoint.at(i) - 0) > DATAMINNUM)
				{
					allMatchLine.at(i).linePoint.assign(robot.radarPoint.begin(), robot.radarPoint.begin() + keyPoint.at(i) - LIMITNUM);
				}
				// 数据部分大于雷达扫到的点的长度
				else if (keyPoint.at(i) > 1080 && keyPoint.at(i + 1) < 1080 && (1080 - keyPoint.at(i + 1)) > DATAMINNUM)
				{
					allMatchLine.at(i).linePoint.assign(robot.radarPoint.begin() + keyPoint.at(i + 1) + LIMITNUM, robot.radarPoint.end());
				}
				// cout << "push point of " << allMatchLine.at(i).wallNum
				// 	 << " data OK(no blind area) " << allMatchLine.at(i).linePoint.size() << endl;
				// 因为没有直线可以用到多于1080个点所以剩下的情况不考虑
			}
			// 跨越了盲区
			else
			{
				if (keyPoint.at(i) > 0 && (keyPoint.at(i) - 0) > (DATAMINNUM / 2))
				{
					allMatchLine.at(i).linePoint.assign(robot.radarPoint.begin(), robot.radarPoint.begin() + keyPoint.at(i) - LIMITNUM);
				}
				// 如果标号没有超过1080
				if (keyPoint.at(i + 1) < 1080 && (1080 - keyPoint.at(i + 1)) > (DATAMINNUM / 2))
				{
					allMatchLine.at(i).linePoint.insert(allMatchLine.at(i).linePoint.end(), robot.radarPoint.begin() + keyPoint.at(i + 1), robot.radarPoint.end());
				}
				// cout << "push point of " << allMatchLine.at(i).wallNum
				// 	 << " data OK(blind area) " << allMatchLine.at(i).linePoint.size() << endl;
			}
		}
		cout << "flag3";
		// 去除不在对应场地直线上的附近点
		for (int i = 0; i < allMatchLine.size(); i++)
		{
			int linePointSize = allMatchLine.at(i).linePoint.size();
			int num = allMatchLine.at(i).wallNum;
			// 筛选直线的点
			for (int j = 0; j < linePointSize; j++)
			{
				float p2WallDistance = P2LineDistance(wallLine.at(num), allMatchLine.at(i).linePoint.at(j));
				// 距离大于所设定的阈值就删除
				if (fabs(p2WallDistance) > 100)
				{
					allMatchLine.at(i).linePoint.erase(allMatchLine.at(i).linePoint.begin() + j);
					// 防止越界，对linePointSize和j都减1，后面有j++相当于j还是对这个位置的数据进行处理
					linePointSize--;
					j--;
				}
				else
				{
					// 在直线上，绘制点
					cv::circle(radarImage, P2Img(allMatchLine.at(i).linePoint.at(j)), 1, cv::Scalar(0, 0, 255), 1);
				}
			}
			// 点数大于20，拟合直线，并且绘制在直线上
			if (allMatchLine.at(i).linePoint.size() > 20)
			{
				robot.recentPosData.at(0);
				allMatchLine.at(i).startPoint = allMatchLine.at(i).linePoint.at(0);
				allMatchLine.at(i).endPoint = allMatchLine.at(i).linePoint.at(allMatchLine.at(i).linePoint.size() - 1);
				LineParam_t line = LineFitLeastSquares(allMatchLine.at(i).linePoint, 0, allMatchLine.at(i).linePoint.size() - 1);
				allMatchLine.at(i).A = line.A;
				allMatchLine.at(i).B = line.B;
				allMatchLine.at(i).C = line.C;
				allMatchLine.at(i).angle = line.angle;
				allMatchLine.at(i).pNum = allMatchLine.at(i).linePoint.size();
				cv::circle(radarImage, P2Img(allMatchLine.at(i).startPoint), 4, cv::Scalar(77, 255, 255));
				cv::circle(radarImage, P2Img(allMatchLine.at(i).endPoint), 4, cv::Scalar(77, 255, 255));
				// cv::line(radarImage, P2Img(allMatchLine.at(i).startPoint), P2Img(allMatchLine.at(i).endPoint), cv::Scalar(0, 0, 255), 2);
			}
			else
			{
				allMatchLine.at(i).angle = 9999;
			}
		}
		cout << "flag4";
		chrono::steady_clock::time_point timeE = chrono::steady_clock::now();
		auto time = chrono::duration_cast<chrono::microseconds>(timeE - timeS);
		// 输出allMatchLine的直线对应的场地直线标号以及其他信息
		// Pos_t robotw = (robot.posData.at(robot.posData.size() - 1) + robot.ppsCorrect);
		cout << "proStamp: " << proStamp << "  " << time.count() / 1000.f << endl;
		// for (int i = 0; i < allMatchLine.size(); i++)
		// {
		// 	cout << "L " << allMatchLine.at(i).wallNum << " lineSize：" << allMatchLine.at(i).linePoint.size() << ' '
		// 		 << allMatchLine.at(i).A << " " << allMatchLine.at(i).B << " " << allMatchLine.at(i).C << ' ' << allMatchLine.at(i).angle << endl;
		// }
		cout << endl;
		//判断机器人内外场
		vector<LineParam_t> lineForCorrect;
		for (int i = 0; i < allMatchLine.size(); i++)
		{
			if (fabs(allMatchLine.at(i).angle - 9999) > 1)
			{
				lineForCorrect.push_back(allMatchLine.at(i));
			}
		}
		for (int i = 0; i < lineForCorrect.size(); i++)
		{
			cout << "L " << lineForCorrect.at(i).wallNum << " lineSize：" << lineForCorrect.at(i).linePoint.size() << ' '
				 << lineForCorrect.at(i).A << " " << lineForCorrect.at(i).B << " " << lineForCorrect.at(i).C << ' ' << lineForCorrect.at(i).angle << endl;
			output << "L " << lineForCorrect.at(i).wallNum << " lineSize：" << lineForCorrect.at(i).linePoint.size() << ' '
				 << lineForCorrect.at(i).A << " " << lineForCorrect.at(i).B << " " << lineForCorrect.at(i).C << ' ' << lineForCorrect.at(i).angle << endl;
		}
		cout << "flag5";
#ifdef DRMODE
		robot.drStatus = PointInPolyTest(cv::Point(robot.rbotPos.x, robot.rbotPos.y), insideZone);
		Pos_t tmpCorrect = robot.ppsCorrect;
		if (lineForCorrect.size() > 0)
		{
			tmpCorrect = CorrectPos(robot.rbotPos, lineForCorrect);
			for (int i = 0; i < lineForCorrect.size(); i++)
			{
				output << "LC " << lineForCorrect.at(i) << ' ' << lineForCorrect.at(i).calXYErr.x
					   << ' ' << lineForCorrect.at(i).calXYErr.y << endl;
				Point p1 = lineForCorrect.at(i).startPoint;
				Point p2 = lineForCorrect.at(i).endPoint;
				// cv::circle(radarImage, P2Img(p1), 4, cv::Scalar(77, 255, 255));
				// cv::circle(radarImage, P2Img(p2), 4, cv::Scalar(77, 255, 255));
				// line(radarImage, P2Img(p1), P2Img(p2), Scalar(75, 255, 255), 1);
			}
		}
		pthread_mutex_lock(&mtxCorrect); //上锁
		robot.ppsCorrect = tmpCorrect;
		pthread_mutex_unlock(&mtxCorrect); //解锁
#endif
#ifdef TRMODE
		Pos_t tmpCorrect = robot.ppsCorrect;
		if (lineForCorrect.size() > 0)
		{
			tmpCorrect = CorrectPos(robot.rbotPos, lineForCorrect);
			output << "tmpCorrect:" << endl;
			// for (int i = 0; i < lineForCorrect.size(); i++)
			// {
			// 	output << "LC " << lineForCorrect.at(i) << ' ' << lineForCorrect.at(i).calXYErr.x
			// 		   << ' ' << lineForCorrect.at(i).calXYErr.y << endl;
			// 	Point p1 = lineForCorrect.at(i).startPoint;
			// 	Point p2 = lineForCorrect.at(i).endPoint;
			// 	cv::circle(radarImage, P2Img(p1), 4, cv::Scalar(77, 255, 255));
			// 	cv::circle(radarImage, P2Img(p2), 4, cv::Scalar(77, 255, 255));
			// 	line(radarImage, P2Img(p1), P2Img(p2), Scalar(75, 255, 255), 1);
			// }
		}
		cout << "flag6";
		pthread_mutex_lock(&mtxCorrect); //上锁
		robot.ppsCorrect = tmpCorrect;
		pthread_mutex_unlock(&mtxCorrect); //解锁
		cout << "flag7";
#endif
	}
	else
	{
		// 没有找到直线，程序逻辑问题
		cerr << "NO FIND WALL!!!" << endl;
	}
}

/*求取vector内数据和
 *@param data: 传入数据
 *@return 容器内数据之和
 */
template <typename T>
T SumVector(vector<T> data)
{
	T sum = std::accumulate(std::begin(data), std::end(data), 0.0);
	return sum;
}
template int SumVector<int>(vector<int> data);
template float SumVector<float>(vector<float> data);

/*求取vector内数据均值
 *@param data: 传入数据
 *@return 容器内数据均值
 */
template <typename T>
T AvgVector(vector<T> data)
{
	T sum = std::accumulate(std::begin(data), std::end(data), 0.0);
	T mean = sum / data.size(); //均值
	return mean;
}
template int AvgVector<int>(vector<int> data);
template float AvgVector<float>(vector<float> data);

/*求取vector内数据均方差
 *@param data: 传入数据
 *@return 容器内数据均方差
 */
template <typename T>
float StdVector(vector<T> data)
{
	float sum = std::accumulate(std::begin(data), std::end(data), 0.0);
	float mean = sum / data.size(); //均值
	float accum = 0.0;
	std::for_each(std::begin(data), std::end(data), [&](const T d) {
		accum += (d - mean) * (d - mean);
	});
	//如果容器内数据量小于2，则返回极大值表示无法求取均方差
	if (data.size() > 1)
	{
		float stdev = sqrt(accum / (data.size() - 1)); //方差
		return stdev;
	}
	else
	{
		return 99999;
	}
}
template float StdVector<float>(vector<float> data);
template float StdVector<int>(vector<int> data);