#include "pos.h"
#include "pps.h"
#include "math.h"
#include "map.h"
#include <algorithm>
#include <iterator>
#include <numeric>
#include <atomic>
#include "data.h"
using namespace cv;
atomic_int AdjustD;
extern vector<LineParam_t> allCalculateLine;
extern vector<LineParam_t> wallLine;
extern Mat radarImage;
extern Robot_t robot;
extern ofstream output;
extern ofstream outangle;
extern ofstream ppsOut;
extern int ErrorMax;
extern int ppsSerial;
extern pthread_mutex_t mtx;
extern pthread_mutex_t mtxCorrect;
extern atomic_bool correctionFlag;
extern bool correctPosFlag;
extern bool ppsSetX;
extern bool ppsSetY;
extern bool InitialFlag;
extern bool InitPosFlag;
extern bool ppsSetA;
extern bool CorrectionAgain;
bool ppsOXOYflag = false;
extern PPSData_t InitialCoordinate;
//存储匹配上的直线
int lineNum = -1;

Point lidarCalPoint;
vector<LineParam_t> insideZone;
using namespace std;
//场地匹配
void MatchMap(void)
{
    vector<LineParam_t> lineForCorrect;
    //cout << robot.recentPosData.at(0).x<< ' ' << robot.recentPosData.at(0).y<<' '<<robot.ppsCorrect.x<<' '<<robot.ppsCorrect.y<< endl;
#ifdef DRMODE
    //判断机器人内外场
    robot.drStatus = PointInPolyTest(cv::Point(robot.rbotPos.x, robot.rbotPos.y), insideZone);
    //遍历所有识别出的直线
    for (int lNum = 0; lNum < allCalculateLine.size(); lNum++)
    {
        //如果DR在外区
        if (robot.drStatus)
        {
            //遍历所有外场的墙
            for (int wNum = 0; wNum < wallLine.size() - 6; wNum++)
            {
                //如果找到匹配的墙则将其标号存入直线参数
                if (JudgeSameLine(allCalculateLine.at(lNum), wallLine.at(wNum)))
                {
                    allCalculateLine.at(lNum).wallNum = wNum;
                    lineForCorrect.push_back(allCalculateLine.at(lNum));
                    break;
                }
            }
        }
        else
        {
            //遍历所有内区的墙
            for (int wNum = 10; wNum < wallLine.size(); wNum++)
            {
                //如果找到匹配的墙则将其标号存入直线参数
                if (JudgeSameLine(allCalculateLine.at(lNum), wallLine.at(wNum)))
                {
                    allCalculateLine.at(lNum).wallNum = wNum;
                    lineForCorrect.push_back(allCalculateLine.at(lNum));
                    break;
                }
            }
        }
        //cout<<allCalculateLine.at(lNum)<<endl;
    }
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
    //遍历所有识别出的直线
    for (int lNum = 0; lNum < allCalculateLine.size(); lNum++)
    {
        //遍历所有外场的墙
        for (int wNum = 0; wNum < wallLine.size(); wNum++)
        {
            if (wNum == 4)
            {
                continue;
            }
            if (wNum == 9 && robot.rbotPos.y > 10000) // 9号墙的点有点问题导致x在要取箭时会有先正矫正100在负矫正100的现象，先暂时用7号墙来矫正x坐标
            {
                continue;
            }
            //如果找到匹配的墙则将其标号存入直线参数
            if (JudgeSameLine(allCalculateLine.at(lNum), wallLine.at(wNum)))
            {
                if (robot.rbotPos.y < 9000 && wNum == 6)
                {
                    break;
                }
                allCalculateLine.at(lNum).wallNum = wNum;
                lineForCorrect.push_back(allCalculateLine.at(lNum));
                break;
            }
        }
    }
    Pos_t tmpCorrect = robot.ppsCorrect;
    if (lineForCorrect.size() > 0)
    {
        tmpCorrect = CorrectPos(robot.rbotPos, lineForCorrect);
        for (int i = 0; i < lineForCorrect.size(); i++)
        {
            // cout << "LC " << lineForCorrect.at(i) << ' ' << lineForCorrect.at(i).calXYErr.x
            //      << ' ' << lineForCorrect.at(i).calXYErr.y << endl;
            output << "LC " << lineForCorrect.at(i) << ' ' << lineForCorrect.at(i).calXYErr.x
                   << ' ' << lineForCorrect.at(i).calXYErr.y << endl;
            Point p1 = lineForCorrect.at(i).startPoint;
            Point p2 = lineForCorrect.at(i).endPoint;
            // cv::circle(radarImage, P2Img(p1), 4, cv::Scalar(77, 255, 255));
            // cv::circle(radarImage, P2Img(p2), 4, cv::Scalar(77, 255, 255));
            // line(radarImage, P2Img(p1), P2Img(p2), Scalar(75, 255, 255), 1);
        }
        if (InitialFlag)
        {

            // 发送矫正后的坐标
            ppsSetX = false;
            while (!ppsSetX)
            {
                CorrectPps(ppsSerial, SETX, (float)(InitialCoordinate.x));
            }
            while (!ppsSetY)
            {
                CorrectPps(ppsSerial, SETY, (float)(InitialCoordinate.y));
            }
            // 重置总误差量
            tmpCorrect.x = 0;
            tmpCorrect.y = 0;
            InitialFlag = false;
            InitPosFlag = false;
            ppsOXOYflag = true;
            cout << "initPos: " << InitialCoordinate.x << ' ' << InitialCoordinate.y << endl;
            output << "initPos: " << InitialCoordinate.x << ' ' << InitialCoordinate.y << endl;
        }
        else if (ppsOXOYflag)
        {
            // 重置总误差量
            tmpCorrect.x = 0;
            tmpCorrect.y = 0;
            ppsOXOYflag = false;
        }
    }
    pthread_mutex_lock(&mtxCorrect); //上锁
    robot.ppsCorrect = tmpCorrect;
    pthread_mutex_unlock(&mtxCorrect); //解锁
#endif
}

Pos_t CorrectPos(Pos_t rbotPos, vector<LineParam_t> &lineForCorrect)
{
    static Pos_t sumCorrectPos;
    vector<int> xSingleCorrect;
    vector<int> ySingleCorrect;
    vector<int> slashLine;
    bool correctFlag = false;
    vector<Pos_t> allCalPos;
    int slashstandardX = 0;
    int slashstandardY = 0;
    int rbot2LineMinDisX = 200000;
    int rbot2LineMinDisY = 200000;
    int MinDisXLineNum = 0;
    int MinDisYLineNum = 0;
    int correctX = 0, correctY = 0;
    static Pos_t lastRbotPos;
    static float correctXMax = 7.0;
    static float correctYMax = 10.0;
    // static float singleXMax = 7.0;
    // static float singleYMax = 10.0;
    for (int i = 0; i < lineForCorrect.size(); i++)
    {
        correctX = 0;
        correctY = 0;
        LineParam_t line = lineForCorrect.at(i);
        int D = fabs(line.A * rbotPos.x + line.B * rbotPos.y + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2));
        //矫正X
        if (wallLine.at(line.wallNum).angle == 90)
        {
            if (robot.rbotPos.x < 1000 && robot.rbotPos.y < 1000)
            {
                CompenStartDis(D); // 补偿x方向
            }
            else
            {
                CompenDistance(D);
            }
            if (rbotPos.x < (-wallLine.at(line.wallNum).C))
            {
                correctX = (-D - wallLine.at(line.wallNum).C) - rbotPos.x;
            }
            else
            {
                correctX = (D - wallLine.at(line.wallNum).C) - rbotPos.x;
            }
            lineForCorrect.at(i).calXYErr = cv::Point(correctX, correctY);

            if (D < rbot2LineMinDisX)
            {
                rbot2LineMinDisX = D;
                MinDisXLineNum = i;
            }
            slashstandardX++;
        }
        //矫正Y
        else if (wallLine.at(line.wallNum).angle == 0)
        {

            if (line.wallNum == 8)
            {
                // if (D > 395 && D < 488)
                // {
                //     D = D + ((-0.0108) * D + 17.247);
                // }
                // else if (D > 488 && D < 585)
                // {
                //     D = D + (0.0309 * D - 3.0928);
                // }
                // else if (D > 585 && D < 682)
                // {
                //     D = D + (0.0309 * D + -3.0928);
                // }
                // else if (D > 682 && D < 780)
                // {
                //     D = D + (0.0204 * D + 4.0816);
                // }
                // else if (D > 780 && D < 878)
                // {
                //     D = D + (0.0204 * D + 4.0816);
                // }
                // else if (D > 878 && D < 978)
                // {
                //     D = D + 22;
                // }
                // else if (D > 978 && D < 1079)
                // {
                //     D = D + ((-0.0099) * D + 31.683);
                // }
                // else if (D > 1079 && D < 1184)
                // {
                //     D = D + ((-0.0476) * D + 72.381);
                // }
                // else if (D > 1184 && D < 1288)
                // {
                //     D = D + ((-0.0385) * D + 61.538);
                // }
                // else if (D > 1288 && D < 1393)
                // {
                //     D = D + ((-0.0476) * D + 73.333);
                // }
                // else if (D > 1393 && D < 1498)
                // {
                //     D = D + (-0.0476 * D + 73.333);
                // }
                D = D + 5;
            }
            else
            {
                CompenDistance(D); // Y方向正常补偿数据
            }

            if (rbotPos.y < (-wallLine.at(line.wallNum).C))
            {
                correctY = (-D - wallLine.at(line.wallNum).C) - rbotPos.y;
            }
            else
            {
                correctY = (D - wallLine.at(line.wallNum).C) - rbotPos.y;
            }
            lineForCorrect.at(i).calXYErr = cv::Point(correctX, correctY);

            if (D < rbot2LineMinDisY)
            {
                rbot2LineMinDisY = D;
                MinDisYLineNum = i;
            }
            slashstandardY++;
        }
        //将匹配上的斜线存入容器
        else if (fabs(wallLine.at(line.wallNum).angle) == 45)
        {
            slashLine.push_back(i);
        }
        lineForCorrect.at(i).rbot2Line = D;
        // output<<"cp "<< i << ' '<< line.wallNum<<' ' <<D<<' '<<wallLine.at(line.wallNum).angle<<' '\
                // <<rbot2LineMinDisX<<' '<<rbot2LineMinDisY<<' '<<MinDisXLineNum<<' '<<MinDisYLineNum<<endl;
    }
    cout << "CXY " << MinDisXLineNum << ' ' << MinDisYLineNum << ' ' << lineForCorrect.size() << endl;
    xSingleCorrect.push_back(lineForCorrect.at(MinDisXLineNum).calXYErr.x);
    ySingleCorrect.push_back(lineForCorrect.at(MinDisYLineNum).calXYErr.y);
    if (slashstandardX != 0 && slashstandardY != 0)
    {
        slashstandardX = 0;
        slashstandardY = 0;
    }
    else if ((slashstandardX >= slashstandardY) && (slashstandardX != 0))
    {
        slashstandardX = 1;
        slashstandardY = 0;
    }
    else if (slashstandardX < slashstandardY)
    {
        slashstandardX = 0;
        slashstandardY = 1;
    }
    correctX = 0, correctY = 0;
    for (int j = 0; j < slashLine.size(); j++)
    {
        int number = slashLine.at(j);
        LineParam_t line = lineForCorrect.at(number);
        int D = fabs(line.A * rbotPos.x + line.B * rbotPos.y + line.C) / sqrt(pow(line.A, 2) + pow(line.B, 2));
        pthread_mutex_lock(&mtxCorrect); //上锁
        AdjustD = D;
        pthread_mutex_unlock(&mtxCorrect); //解锁
        int correctX = 0, correctY = 0;
        if (fabs(wallLine.at(line.wallNum).angle) == 45 && slashstandardY == 1)
        {
            float liney = (-wallLine.at(line.wallNum).C) / wallLine.at(line.wallNum).B +
                          ((-wallLine.at(line.wallNum).A) / wallLine.at(line.wallNum).B) * rbotPos.x;
            float linex = (-wallLine.at(line.wallNum).C) / wallLine.at(line.wallNum).A +
                          ((-wallLine.at(line.wallNum).B) / wallLine.at(line.wallNum).A) * rbotPos.y;
            float Dx = D * sqrt(2);
            float Dy = Dx;
            if (rbotPos.x > linex)
            {
                correctX = ((Dx + linex) - rbotPos.x);
                xSingleCorrect.push_back(correctX);
            }
            else
            {
                correctX = ((-Dx + linex) - rbotPos.x);
                xSingleCorrect.push_back(correctX);
            }
        }
        else if (fabs(wallLine.at(line.wallNum).angle) == 45 && slashstandardX == 1)
        {
            float liney = (-wallLine.at(line.wallNum).C) / wallLine.at(line.wallNum).B +
                          ((-wallLine.at(line.wallNum).A) / wallLine.at(line.wallNum).B) * rbotPos.x;
            float linex = (-wallLine.at(line.wallNum).C) / wallLine.at(line.wallNum).A +
                          ((-wallLine.at(line.wallNum).B) / wallLine.at(line.wallNum).A) * rbotPos.y;
            float Dx = D * sqrt(2);
            float Dy = Dx;
            if (rbotPos.y > liney)
            {
                correctY = ((Dy + liney) - rbotPos.y);
                ySingleCorrect.push_back(correctY);
            }
            else
            {
                correctY = ((-Dy + liney) - rbotPos.y);
                ySingleCorrect.push_back(correctY);
            }
        }
        lineForCorrect.at(number).calXYErr = cv::Point(correctX, correctY);
    }
    Pos_t ppsErr;
    if (xSingleCorrect.size() > 0)
    {
        float sum = accumulate(xSingleCorrect.begin(), xSingleCorrect.end(), 0);
        ppsErr.x = sum / xSingleCorrect.size();
    }
    if (ySingleCorrect.size() > 0)
    {
        float sum = accumulate(ySingleCorrect.begin(), ySingleCorrect.end(), 0);
        ppsErr.y = sum / ySingleCorrect.size();
    }
    robot.ppsErr = ppsErr;
    //CorrectXMax = robot.posData.at(5).x - robot.posData.at(4).x;
    //限幅
    // cout << "posdatas:" << robot.posData.size() << endl;
    // cout << "flag3"<<endl;
    // if(correctionFlag)
    // {
    //     CorrectXMax = (robot.posData.at(5).x - robot.posData.at(4).x) * 0.5;
    // }
    cout << "flag4" << endl;
    //根据速度来个改变单次矫正量阈值
    // Pos_t differPos;
    // differPos.x = fabs(rbotPos.x-lastRbotPos.x);
    // differPos.y = fabs(rbotPos.y-lastRbotPos.y);
    // if(correctionFlag)
    // {
    //     static int continueFlag = 0;
    //     if(!continueFlag)
    //     {
    //         correctXMax = differPos.x * 0.1 +2;
    //         correctYMax = differPos.y * 0.1 +2;
    //         if(correctXMax > 10)
    //         {
    //             correctXMax = 10;
    //         }
    //         if(correctYMax > 13)
    //         {
    //             correctYMax = 13;
    //         }
    //         if(correctXMax > 7)
    //         {
    //             continueFlag = 2;
    //         }
    //         else if(correctYMax > 10)
    //         {
    //             continueFlag = 2;
    //         }
    //     }
    //     else
    //     {
    //         if(correctXMax > 7)
    //         {
    //             continueFlag--;
    //         }
    //         else if(correctYMax > 10)
    //         {
    //             continueFlag--;
    //         }
    //     }
    // }
    // //lastRbotPos中存入上一次的坐标
    // lastRbotPos = rbotPos;
    // output<<"correct"<<" "<<correctXMax<<" "<<correctYMax<<" "<<differPos.x<<" "<<differPos.y<<endl;
    if (fabs(ppsErr.x) > ErrorMax /*|| robot.rbotVel.x * ppsErr.x < 0*/)
    {
        robot.singleCorrect.x = 0;
    }
    else if (fabs(ppsErr.x) > correctXMax)
    {
        if (ppsErr.x > 0)
        {
            robot.singleCorrect.x = correctXMax;
        }
        else
        {
            robot.singleCorrect.x = -correctXMax;
        }
    }
    else
    {
        robot.singleCorrect.x = ppsErr.x;
    }
    // cout << "flag5"<<endl;

    //CorrectYMax = robot.posData.at(5).y - robot.posData.at(4).y;
    // if(correctionFlag)
    // {
    //     CorrectYMax = (robot.posData.at(5).y - robot.posData.at(4).y) * 0.5;
    // }
    // cout << "flag6"<<endl;

    if (fabs(ppsErr.y) > ErrorMax /*|| robot.rbotVel.y * ppsErr.y < 0*/)
    {
        robot.singleCorrect.y = 0;
    }
    else if (fabs(ppsErr.y) > correctYMax)
    {
        if (ppsErr.y > 0)
        {
            robot.singleCorrect.y = correctYMax;
        }
        else
        {
            robot.singleCorrect.y = -correctYMax;
        }
    }
    else
    {
        robot.singleCorrect.y = ppsErr.y;
    }
    correctPosFlag = SetPosFlag();
    // 如果在取箭区附近就开始矫正，即加上这一次的矫正量
    if (correctPosFlag)
    {
        sumCorrectPos.x = sumCorrectPos.x + robot.singleCorrect.x;
        sumCorrectPos.y = sumCorrectPos.y + robot.singleCorrect.y;
    }
    else
    {
        sumCorrectPos.x = sumCorrectPos.x;
        // sumCorrectPos.x = sumCorrectPos.x + robot.singleCorrect.x;
        sumCorrectPos.y = sumCorrectPos.y;
    }
    // cout << "C: "<<sumCorrectPos << "  "<<ppsErr<< endl;
    return sumCorrectPos;
}

/*判断点是否在多边形内部
 *@param point 被判断点的坐标
 *@param lines 多边形的各个边参数
 *@return 如果在多边形外部则返回true 在多边形内部则返回false
 */
bool PointInPolyTest(cv::Point point, vector<LineParam_t> lines)
{
    int crossNum = 0;
    for (int i = 0; i < lines.size(); i++)
    {
        //只判断不垂直于x轴的直线
        if (lines[i].slope < 999)
        {
            //判断是否在直线x坐标中间
            bool cond1 = (lines[i].startPoint.x < point.x) && (point.x < lines[i].endPoint.x);
            bool cond2 = (lines[i].endPoint.x < point.x) && (point.x < lines[i].startPoint.x);
            //判断是否在直线下方
            bool under = (point.y < (lines[i].slope * point.x - lines[i].C / lines[i].B));
            if ((cond1 || cond2) && under)
            {
                crossNum++;
            }
        }
    }

    return (crossNum % 2 == 0);
}

bool CorrectAngle(vector<LineParam_t> lines, float maxLineAngErr)
{
    float posAngleErr = 0.0;
    static float posAngleErrSum;
    static int cnt = 0;
    int errCnt = 0;
    LineParam_t tmpLine;
    float tmpLineAng;
    float tmpAng;
    int tmpLenth;

    //存放每条直线计算的角度误差
    vector<float> angErr;
    //存放对应直线的长度
    vector<int> linesLenth;
    for (int i = 0; i < lines.size(); i++)
    {
        outangle << lines.at(i) << endl;
        tmpLine = lines.at(i);
        tmpLineAng = tmpLine.angle;
        tmpLenth = sqrt(pow((tmpLine.startPoint.x - tmpLine.endPoint.x), 2) + pow((tmpLine.startPoint.y - tmpLine.endPoint.y), 2));
        //90度直线
        if (fabs(tmpLineAng - 90) < maxLineAngErr)
        {
            tmpAng = 90 - tmpLineAng;
            angErr.push_back(tmpAng);
            linesLenth.push_back(tmpLenth);
        }
        //-90度直线
        else if (fabs(tmpLineAng + 90) < maxLineAngErr)
        {
            tmpAng = -90 - tmpLineAng;
            angErr.push_back(tmpAng);
            linesLenth.push_back(tmpLenth);
        }
        //0度平行直线
        else if (fabs(tmpLineAng - 0) < maxLineAngErr)
        {
            tmpAng = 0 - tmpLineAng;
            angErr.push_back(tmpAng);
            linesLenth.push_back(tmpLenth);
        }
        //45度直线
        else if (fabs(tmpLineAng - 45) < maxLineAngErr)
        {
            tmpAng = 45 - tmpLineAng;
            angErr.push_back(tmpAng);
            linesLenth.push_back(tmpLenth);
        }
        //-45度直线
        else if (fabs(tmpLineAng + 45) < maxLineAngErr)
        {
            tmpAng = -45 - tmpLineAng;
            angErr.push_back(tmpAng);
            linesLenth.push_back(tmpLenth);
        }
    }
    //至少找到两条直线用于矫正角度
    int tmpSize = angErr.size();
    if (tmpSize > 1)
    {
        int lenthSum = SumVector(linesLenth);
        for (int i = 0; i < tmpSize; i++)
        {
            float tmp = (float)linesLenth.at(i) / (float)lenthSum;
            outangle << "ang " << angErr.at(i) << ' ' << linesLenth.at(i) << ' ' << tmp << ' ';
            angErr.at(i) = angErr.at(i) * tmp;
            outangle << angErr.at(i) << endl;
        }
        //方差较小
        if (StdVector(angErr) < 2)
        {
            posAngleErr = SumVector<float>(angErr);
            posAngleErrSum += posAngleErr;
            outangle << "SC " << cnt << "： " << posAngleErr << endl;
            cnt++;
        }
    }

    if (cnt > 30)
    {
        posAngleErr = posAngleErrSum / cnt;
        if (!CorrectionAgain)
        {
            pthread_mutex_lock(&mtx);
            robot.ppsInitAngle = robot.ppsInitAngle + posAngleErr;
            robot.lidarRotation = robot.ppsInitAngle + 90;
            pthread_mutex_unlock(&mtx);
            CorrectionAgain = !CorrectionAgain;
            outangle << " CorrectionAgain"
                     << " " << robot.ppsInitAngle << endl;
        }
        else
        {
            cout << "CorrectPps SETA: " << (float)(posAngleErr - 90) << endl;
            //更新定位系统初始角度
            while (!ppsSetA)
            {
                // ppsOut << "111111111111111111" << ppsSetA << endl;
                CorrectPps(ppsSerial, SETA, (float)(posAngleErr - 90));
            }
        }
        cnt = 0;
        outangle << "CorrectPps SETA ok : " << posAngleErr << endl;
        output << "CorrectPps SETA ok : " << (posAngleErr - 90) << endl;
        output << "CorrectPps InitAng ok : " << (posAngleErr - 90) << endl;
        posAngleErrSum = 0;
        return 1; //1为posXY模式
    }
    return 0; //0为posAng模式
}

//除了初始位置时，用雷达矫正坐标的情况
bool SetPosFlag()
{
#ifdef TRMODE
    // TR在取箭之后射箭时进行矫正坐标
    bool corFlag1 = (robot.rbotPos.x > 1300 && robot.rbotPos.y < 6200);
    // TR快要取箭时进行矫正坐标
    bool corFlag2 = (robot.rbotPos.y > 7500 && robot.rbotPos.x > 800);
    bool corFlag3 = (robot.rbotPos.x < 1000 && robot.rbotPos.y < 1000);
    if (corFlag1 || corFlag2 || corFlag3)
    //if ((robot.rbotPos.x < 2500 && robot.rbotPos.y > 10800) || (robot.rbotPos.x <1000 && robot.rbotPos.y <1000))
    {
        return true;
    }
    else
    {
        return false;
    }
#endif
    return true;
}

// 补偿车到墙计算出的距离值
void CompenDistance(int &distance)
{
    if (distance >= 880 && distance < 1678)
    {
        distance = 0.9045 * (float)distance + 64.226;
    }
    else if (distance > 430 && distance < 880)
    {
        distance = 1.0198 * (float)distance - 8.2369;
    }
}
// 补偿初始位x方向
void CompenStartDis(int &distance)
{
    if (distance > 460 && distance < 580)
    {
        distance = 1.0857 * (float)distance - 14.801;
    }
}
