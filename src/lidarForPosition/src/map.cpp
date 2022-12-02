#include "map.h"
#include "pos.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
extern const uint8_t L_NUME;

using namespace std;
using namespace cv;

extern vector<LineParam_t> wallLine;
extern vector<LineParam_t> insideZone;
extern vector<Point> pathPointRead;
//extern vector<LineParam_t> allLine(30);

//初始化场地直线参数与
void InitMap(void)
{
	//外场
	vector<LineParam_t> allLine(30);
	//y=0
	allLine[0].A = 0;
	allLine[0].B = 1;
	allLine[0].C = 0;
	allLine[0].slope = 0;
	allLine[0].angle = 0;
	//x=5925
	allLine[1].A = 1;
	allLine[1].B = 0;
	allLine[1].C = -5925;
	allLine[1].slope = 9999;
	allLine[1].angle = 90;
	//y=1950
	allLine[2].A = 0;
	allLine[2].B = 1;
	allLine[2].C = -1950;
	allLine[2].slope = 0;
	allLine[2].angle = 0;
	//x+y=5900
	allLine[3].A = 1;
	allLine[3].B = 1;
	allLine[3].C = -5900;
	allLine[3].slope = -1;
	allLine[3].angle = -45;
	//x=1950
	allLine[4].A = 1;
	allLine[4].B = 0;
	allLine[4].C = -1950;
	allLine[4].slope = 9999;
	allLine[4].angle = 90;
	//x-y+6000=0
	allLine[5].A = 1;
	allLine[5].B = -1;
	allLine[5].C = 6000;
	allLine[5].slope = 1;
	allLine[5].angle = 45;
	//y=9950
	allLine[6].A = 0;
	allLine[6].B = 1;
	allLine[6].C = -9950;
	allLine[6].slope = 0;
	allLine[6].angle = 0;
	//x=5925
	allLine[7].A = 1;
	allLine[7].B = 0;
	allLine[7].C = -5925;
	allLine[7].slope = 9999;
	allLine[7].angle = 90;
	//y=11900
	allLine[8].A = 0;
	allLine[8].B = 1;
	allLine[8].C = -11900;
	allLine[8].slope = 0;
	allLine[8].angle = 0;
	//x=0
	allLine[9].A = 1;
	allLine[9].B = 0;
	allLine[9].C = 0;
	allLine[9].slope = 9999;
	allLine[9].angle = 90;

	//y=5975
	allLine[10].A = 0;
	allLine[10].B = 1;
	allLine[10].C = -5975;
	allLine[10].slope = 0;
	allLine[10].angle = 0;

	/*起始点和终止点*/
	//外场
	allLine[0].startPoint.x = 0;
	allLine[0].startPoint.y = 0;
	allLine[0].endPoint.x = 5925;
	allLine[0].endPoint.y = 0;

	allLine[1].startPoint.x = allLine[0].endPoint.x;
	allLine[1].startPoint.y = allLine[0].endPoint.y;
	allLine[1].endPoint.x = 5925;
	allLine[1].endPoint.y = 1950;

	allLine[2].startPoint.x = allLine[1].endPoint.x;
	allLine[2].startPoint.y = allLine[1].endPoint.y;
	allLine[2].endPoint.x = 3950;
	allLine[2].endPoint.y = 1950;

	allLine[3].startPoint.x = allLine[2].endPoint.x;
	allLine[3].startPoint.y = allLine[2].endPoint.y;
	allLine[3].endPoint.x = 1950;
	allLine[3].endPoint.y = 3950;

	allLine[4].startPoint.x = allLine[3].endPoint.x;
	allLine[4].startPoint.y = allLine[3].endPoint.y;
	allLine[4].endPoint.x = 1950;
	allLine[4].endPoint.y = 7950;

	allLine[5].startPoint.x = allLine[4].endPoint.x;
	allLine[5].startPoint.y = allLine[4].endPoint.y;
	allLine[5].endPoint.x = 3950;
	allLine[5].endPoint.y = 9950;

	allLine[6].startPoint.x = allLine[5].endPoint.x;
	allLine[6].startPoint.y = allLine[5].endPoint.y;
	allLine[6].endPoint.x = 5925;
	allLine[6].endPoint.y = 9950;

	allLine[7].startPoint.x = allLine[6].endPoint.x;
	allLine[7].startPoint.y = allLine[6].endPoint.y;
	allLine[7].endPoint.x = 5925;
	allLine[7].endPoint.y = 11900;

	allLine[8].startPoint.x = allLine[7].endPoint.x;
	allLine[8].startPoint.y = allLine[7].endPoint.y;
	allLine[8].endPoint.x = 0;
	allLine[8].endPoint.y = 11900;

	allLine[9].startPoint.x = allLine[8].endPoint.x;
	allLine[9].startPoint.y = allLine[8].endPoint.y;
	allLine[9].endPoint.x = 0;
	allLine[9].endPoint.y = 0;

	allLine[10].startPoint.x = 2000;
	allLine[10].startPoint.y = 5975;
	allLine[10].endPoint.x = 9900;
	allLine[10].endPoint.y = 5975;
#ifdef DRMODE
	// 内场
	//y=5975
	allLine[10].A = 0;
	allLine[10].B = 1;
	allLine[10].C = -5975;
	allLine[10].slope = 0;
	allLine[10].angle = 0;
	//x=9900
	allLine[11].A = 1;
	allLine[11].B = 0;
	allLine[11].C = -9900;
	allLine[11].slope = 9999;
	allLine[11].angle = 90;
	//x+y-17850=0
	allLine[12].A = 1;
	allLine[12].B = 1;
	allLine[12].C = -17829.29;
	allLine[12].slope = -1;
	allLine[12].angle = -45;
	//y=9900
	allLine[13].A = 0;
	allLine[13].B = 1;
	allLine[13].C = -9900;
	allLine[13].slope = 0;
	allLine[13].angle = 0;
	//x-y+5950=0
	allLine[14].A = 1;
	allLine[14].B = -1;
	allLine[14].C = 5929.29;
	allLine[14].slope = 1;
	allLine[14].angle = 45;
	//x=2000
	allLine[15].A = 1;
	allLine[15].B = 0;
	allLine[15].C = -2000;
	allLine[15].slope = 9999;
	allLine[15].angle = 90;
	/*起始点和终止点*/
	allLine[10].startPoint.x = 2000;
	allLine[10].startPoint.y = 5975;
	allLine[10].endPoint.x = 9900;
	allLine[10].endPoint.y = 5975;

	allLine[11].startPoint.x = allLine[10].endPoint.x;
	allLine[11].startPoint.y = allLine[10].endPoint.y;
	allLine[11].endPoint.x = 9900;
	allLine[11].endPoint.y = 7929;

	allLine[12].startPoint.x = allLine[11].endPoint.x;
	allLine[12].startPoint.y = allLine[11].endPoint.y;
	allLine[12].endPoint.x = 7929;
	allLine[12].endPoint.y = 9900;

	allLine[13].startPoint.x = allLine[12].endPoint.x;
	allLine[13].startPoint.y = allLine[12].endPoint.y;
	allLine[13].endPoint.x = 3971;
	allLine[13].endPoint.y = 9900;

	allLine[14].startPoint.x = allLine[13].endPoint.x;
	allLine[14].startPoint.y = allLine[13].endPoint.y;
	allLine[14].endPoint.x = 2000;
	allLine[14].endPoint.y = 7929;

	allLine[15].startPoint.x = allLine[14].endPoint.x;
	allLine[15].startPoint.y = allLine[14].endPoint.y;
	allLine[15].endPoint.x = 2000;
	allLine[15].endPoint.y = 5975;

	//代表结束场地直线传入
	allLine[16].A = 9999;
#endif
#ifdef TRMODE
	//代表结束场地直线传入
	allLine[11].A = 9999;
#endif
	cout << "init map......" << endl;
	for (int i = 0; allLine[i].A < 9999; ++i)
	{
		cout << i << endl;
		wallLine.push_back(allLine.at(i));
	}
#ifdef DRMODE
	{
		insideZone.assign(wallLine.end() - 5, wallLine.end());
	}
#endif
	cout << "map ready" << endl;
}

/** @brief 将世界坐标系中点的坐标转换到图像坐标系中
 * @param p 点的坐标
 * @param origin 图像中的相对原点
 * @param scale 缩小比例
*/
cv::Point P2Img(cv::Point p, int originX, int originY, int scale)
{
	cv::Point ret;
	/*激光雷达是逆时针进行旋转的，Mat图像以左上角为原点，向右、下方为正。
	但是我们观察图像当然是以激光雷达为中心，y轴向上为正，所以这里y得取相反数*/
	ret.x = (int)(p.x / scale) + originX;
	ret.y = (int)((-p.y / scale) + originY);
	return ret;
}

//画出场地的图像
void ShowMap(Mat mapImage, vector<LineParam_t> wallLine)
{
	Point startImagePoint;
	Point endImagePoint;
	//设场地坐标原点在图像坐标的（700,700）  按1:scale绘制
	for (int lN = 0; lN < wallLine.size(); lN++)
	{
#ifndef TEST_MODE
		startImagePoint = P2Img(wallLine[lN].startPoint);
		endImagePoint = P2Img(wallLine[lN].endPoint);
#endif
		//画出每条直线 颜色为橙色 粗细2
		// cout<<startImagePoint.x<<" "<<startImagePoint.y<<" ";
		// cout<<endImagePoint.x<<" "<<endImagePoint.y<<endl;
		cv::line(mapImage, startImagePoint, endImagePoint, cv::Scalar(255), 2);
		Point pt1, pt2;
		pt1 = cv::Point(100, 700);
		pt2 = cv::Point(140, 700);
		arrowedLine(mapImage, pt1, pt2, Scalar(34, 34, 178), 2);
		pt2 = cv::Point(100, 660);
		arrowedLine(mapImage, pt1, pt2, Scalar(0, 139, 69), 2);
	}
	//显示上次路线
	ifstream pathRead;
	pathRead.open("/home/action/code/2021/TR/ABU2021_PC_TR/build/pathXY.txt", ios::in);
	if (!pathRead.is_open())
	{
		cout << "没有找到文件" << endl;
		return;
	}
	string line;
	int readflag = 0;
	float num1 = 0.0, num2 = 0.0;
	cout << "读取数据..." << endl;
	while (!pathRead.eof())
	{
		//cout<<num1<<' '<<num2<<endl;
		pathRead >> num1 >> num2;
		pathPointRead.push_back(Point(num1, num2));
		readflag = 1;
	}
	pathRead.close();
	if (readflag)
	{
		for (int i = 0; i < pathPointRead.size() - 1; i++)
		{
			pathPointRead.at(i) = P2Img(pathPointRead.at(i));
			cv::circle(mapImage, pathPointRead[i], 1, Scalar(0, 0, 255), -1);
		}
		readflag = 0;
	}
//显示规划路径
#ifdef DRMODE
	extern Pos_t DRDefense[];
	extern Pos_t DRAttack[];
	for (int i = 0; DRDefense[i + 1].x != 0 && DRAttack[i + 1].x != 0; i++)
	{
		if (DRDefense[i + 1].x != 0)
		{
			Point p1(DRDefense[i].x, DRDefense[i].y);
			Point p2(DRDefense[i + 1].x, DRDefense[i + 1].y);
			cv::line(mapImage, P2Img(p1), P2Img(p2), cv::Scalar(255, 255, 255), 1);
		}
	}
	for (int i = 0; DRAttack[i] != 0; i++)
	{
		if (DRAttack[i + 1].x != 0)
		{
			cv::circle(mapImage, P2Img(Point(DRAttack[i].x, DRAttack[i].y)), 1, cv::Scalar(255, 255, 255), 1);
		}
	}
#endif
#ifdef TRMODE
	extern Pos_t TRAttack[];
	for (int i = 0; TRAttack[i + 1].x != 0; i++)
	{
		if (TRAttack[i + 1].x != 0)
		{
			Point p1(TRAttack[i].x, TRAttack[i].y);
			Point p2(TRAttack[i + 1].x, TRAttack[i + 1].y);
			// cv::line(mapImage, P2Img(p1), P2Img(p2), cv::Scalar(255, 255, 255), 1);
		}
	}
#endif
}

#ifdef DRMODE
Pos_t DRDefense[] =
	{
		//第二段：进攻射箭
		{630, 11400, 180},
		{662.4, 11400, 180},
		{756, 11400, 180},
		{895.5, 11385.7, -179.4781875},
		{1060.8, 11326.2, -178.3781085},
		{1243.3, 11205.8, -178.7218832},
		{1442.1, 11029.6, 177.9263137},
		{1658.7, 10804.9, 173.3598401},
		{1892.8, 10556.6, 166.1921381},
		{2134.4, 10337.9, 153.879275},
		{2363.8, 10180.5, 142.2482318},
		{2557.9, 10072.4, 136.4556285},
		{2700.4, 9996.8, 135},
		{2785.8, 9950, 135},
		{2814, 9934, 135},
		//第三段：回去取箭
		{2814, 9934, 135},
		{2789.1, 9958, 138.87924},
		{2718.2, 10035, 148.9575676},
		{2611.2, 10173.5, 161.6371236},
		{2476.5, 10374.4, 173.6577781},
		{2316.2, 10630.8, -177.8739057},
		{2130.6, 10907.6, -175.0033871},
		{1919.8, 11142.7, -176.2524351},
		{1683.1, 11301.3, -178.3781085},
		{1430.8, 11380.3, -179.6558044},
		{1181.7, 11400, 180},
		{956.3, 11400, 180},
		{778.2, 11400, 180},
		{667.1, 11400, 180},
		{630, 11400, 180},
};
Pos_t DRAttack[] =
	{
		//去防第一个壶轨迹
		{5330, 11541, 150},
		{5294.9, 11519.4, 149.9888916},
		{5191.1, 11452.3, 149.943055},
		{5020.6, 11336.7, 149.8857592},
		{4783.8, 11172, 149.8685705},
		{4480.6, 10958.4, 149.9086775},
		{4111.4, 10695.6, 150.051917},
		{3675.6, 10384.4, 150.4415283},
		{3173, 10025.8, 151.0947002},
		{2627.3, 9622.1, 149.8284634},
		{2107.4, 9186.2, 144.3681756},
		{1689.6, 8730.1, 134.7539438},
		{1410.5, 8263.4, 121.570185},
		{1279, 7813.5, 106.8394401},
		{1280.2, 7405.3, 95.71259968},
		{1359.2, 7052.1, 91.69043595},
		{1470.9, 6769.9, 91.24352887},
		{1609.9, 6566.1, 90.83672884},
		{1773.1, 6442.1, 90.42992881},
		{1950, 6400.5, 90.00021046},
		{2000, 6400.5, 90.00021046},
		{2122.5, 6402.1, 89.42152309},
		{2257.5, 6412, 87.77713421},
		{2391.6, 6440.2, 86.17285239},
		{2516.6, 6491.7, 86.1442045},
		{2626.5, 6557.3, 87.35887502},
		{2714.1, 6619.6, 88.45322441},
		{2775.7, 6667.4, 89.16369208},
		{2812, 6697, 89.74810903},
		{2824, 6707, 90},
		//去防第二个壶轨迹
		{5330, 11541, 150},
		{5294.9, 11519.4, 149.9888916},
		{5191.1, 11452.3, 149.943055},
		{5020.6, 11336.7, 149.8857592},
		{4783.8, 11172, 149.8685705},
		{4480.6, 10958.4, 149.9086775},
		{4111.4, 10695.6, 150.051917},
		{3675.6, 10384.4, 150.4415283},
		{3173, 10025.8, 151.0947002},
		{2627.3, 9622.1, 149.8284634},
		{2107.4, 9186.2, 144.3681756},
		{1689.6, 8730.1, 134.7539438},
		{1410.5, 8263.4, 121.570185},
		{1279, 7813.5, 106.8394401},
		{1280.2, 7405.3, 95.71259968},
		{1359.2, 7052.1, 91.69043595},
		{1470.9, 6769.9, 91.24352887},
		{1609.9, 6566.1, 90.83672884},
		{1773.1, 6442.1, 90.42992881},
		{1950, 6400.5, 90.00021046},
		{2000, 6400.5, 90.00021046},
		{2123.5, 6404.5, 88.51624977},
		{2255.7, 6426.2, 84.49981563},
		{2394.4, 6482.8, 78.88482923},
		{2537.2, 6577.4, 73.90009642},
		{2691.1, 6694.8, 72.40467657},
		{2870.8, 6817.5, 74.77672184},
		{3088.6, 6928.4, 79.56664901},
		{3347.2, 7008.1, 86.1442045},
		{3636.6, 7038.3, 93.94788967},
		{3934.9, 7014.8, 100.2275071},
		{4219.3, 6958, 102.5880932},
		{4475.6, 6894.5, 102.0094058},
		{4698.9, 6837.7, 100.1759409},
		{4888.8, 6791.8, 97.84400267},
		{5044.9, 6757.6, 95.48914614},
		{5166.9, 6733.8, 93.3405544},
		{5254.1, 6718.4, 91.58730355},
		{5306.5, 6709.8, 90.41274007},
		{5324, 6707, 90},
		//去防第三个壶轨迹
		{5330, 11541, 150},
		{5294.9, 11519.4, 149.9888916},
		{5191.1, 11452.3, 149.943055},
		{5020.6, 11336.7, 149.8857592},
		{4783.8, 11172, 149.8685705},
		{4480.6, 10958.4, 149.9086775},
		{4111.4, 10695.6, 150.051917},
		{3675.6, 10384.4, 150.4415283},
		{3173, 10025.8, 151.0947002},
		{2627.3, 9622.1, 149.8284634},
		{2107.4, 9186.2, 144.3681756},
		{1689.6, 8730.1, 134.7539438},
		{1410.5, 8263.4, 121.570185},
		{1279, 7813.5, 106.8394401},
		{1280.2, 7405.3, 95.71259968},
		{1359.2, 7052.1, 91.69043595},
		{1470.9, 6769.9, 91.24352887},
		{1609.9, 6566.1, 90.83672884},
		{1773.1, 6442.1, 90.42992881},
		{1950, 6400.5, 90.00021046},
		{2000, 6400.5, 90.00021046},
		{2166.4, 6408.4, 86.62548905},
		{2341.8, 6453.2, 79.41767998},
		{2533.2, 6562.7, 73.67664288},
		{2754.4, 6727.3, 72.98336394},
		{3029, 6904.5, 77.95663761},
		{3379.2, 7047, 86.28744395},
		{3811, 7127.5, 95.43757993},
		{4314.4, 7138.5, 103.161051},
		{4867.9, 7092.2, 107.2405105},
		{5429.1, 7019, 108.0140035},
		{5948.8, 6943.6, 107.6702289},
		{6404.4, 6874.3, 107.1144598},
		{6794.3, 6814.4, 105.9227076},
		{7119.1, 6768.4, 103.8715187},
		{7378.8, 6738, 101.6197945},
		{7572.9, 6720.2, 99.64881973},
		{7707.9, 6710.6, 96.30847578},
		{7793.2, 6707.3, 91.95972612},
		{7824, 6707, 90},
};
#endif
#ifdef TRMODE
Pos_t TRAttack[] =
	{
		//第一段：进攻取箭
		{580, 485, 150},
		{577.2, 548.9, 150.41861},
		{565.6, 740.2, 152.2062383},
		{546.5, 1058.6, 155.4778273},
		{531.7, 1504.6, 158.9957881},
		{530, 2078.6, 161.7574447},
		{542, 2780, 163.6825829},
		{562.8, 3608.5, 165.0691408},
		{584.1, 4563.3, 166.0374395},
		{601.4, 5625.4, 166.6046677},
		{616.5, 6713, 167.0401156},
		{627.4, 7718.3, 167.6245325},
		{639.1, 8596.7, 169.383513},
		{676.1, 9346.6, 173.6692373},
		{743.9, 9971.9, 178.0752827},
		{812.6, 10484.5, -179.919365},
		{868.8, 10886.6, -179.7302889},
		{926.4, 11172.2, 180},
		{978.8, 11342.7, 180},
		{1000, 11400, 180},
		//第二段：进攻取箭
		{1000, 11400, 180},
		{979.4444444, 11400, 180},
		{958.8888889, 11400, 180},
		{938.3333333, 11400, 180},
		{917.7777778, 11400, 180},
		{897.2222222, 11400, 180},
		{876.6666667, 11400, 180},
		{856.1111111, 11400, 180},
		{835.5555556, 11400, 180},
		{815, 11400, 180},
		{794.4444444, 11400, 180},
		{773.8888889, 11400, 180},
		{753.3333333, 11400, 180},
		{732.7777778, 11400, 180},
		{712.2222222, 11400, 180},
		{691.6666667, 11400, 180},
		{671.1111111, 11400, 180},
		{650.5555556, 11400, 180},
		{630, 11400, 180},
		//第三段：进攻射箭
		{630, 11400, 180},
		{655.8, 11400, 180},
		{719.9, 11400, 180},
		{807.1, 11381.9, 179.7769674},
		{910.9, 11308.8, 179.3587082},
		{1023.5, 11161.5, 178.8831532},
		{1139.3, 10940.5, 177.5596207},
		{1255.6, 10649.6, 174.826612},
		{1384.7, 10307.2, 167.4182677},
		{1546.1, 9969.8, 154.0798103},
		{1728.1, 9695.5, 142.2138543},
		{1892.1, 9491.7, 136.490006},
		{2012.5, 9346, 135},
		{2083.7, 9257, 135},
		{2107, 9227, 135},
		//第四段：回去取箭
		{2107, 9227, 135},
		{2082.6, 9266.5, 139.5209527},
		{2019.1, 9391.2, 150.865517},
		{1938.1, 9610.4, 164.4675351},
		{1851.6, 9921.4, 175.8292882},
		{1753.6, 10300.5, -177.4957535},
		{1637.4, 10687.8, -175.0033871},
		{1500, 11011.2, -175.0033871},
		{1340.2, 11238.1, -176.132114},
		{1164.6, 11363.4, -178.4239451},
		{989.1, 11400, 180},
		{835, 11400, 180},
		{720.5, 11400, 180},
		{652.4, 11400, 180},
		{630, 11400, 180},
};
#endif