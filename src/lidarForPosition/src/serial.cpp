#include "serial.h"
#include "pps.h"
//#include "data.h"
#include <atomic>
//进程通讯头文件
extern "C"
{
#include "process_comm.h"
}
pthread_mutex_t mtx;
pthread_mutex_t mtxCorrect;
extern Robot_t robot;
extern atomic_bool correctionFlag;
//定位系统矫正值
// atomic_int ppsCorrect_fx;
// atomic_int ppsCorrect_fy;
int rerDirFlg = 8;
int correctFlag = 8;
atomic_bool ppsFlag;
bool ppsSetX = false;
bool ppsSetY = false;
bool ppsSetA = false;
bool ppsSetR = false;
bool CorrectionAgain = true;
float R2Wall10Dis = 0.0;
PPSData_t InitialCoordinate;
extern atomic_bool runFlag;
// extern atomic_bool correctionFlag;
extern bool InitialFlag;
extern bool InitPosFlag;
extern bool correctPosFlag;
extern atomic_int proStamp;
extern ofstream outTime;
extern int ppsSerial;
// extern int correctionflag;
extern ofstream output;

ofstream ppsOut;
//extern int mcuSerial;
own_serial::own_serial()
{
}
own_serial::~own_serial()
{
}

int own_serial::Init(int &fd, char *port, int baudrate)
{
    std::cout << "init " << port << " ...\n";
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    std::cout << fd << std::endl;
    if (fd == -1)
    {
        perror("open_port: Unable to open\n");
        return 0;
    }
    tcgetattr(fd, &termAttr);
    bzero(&termAttr, sizeof(termAttr));
    //设置波特率
    cfsetispeed(&termAttr, baudrate);
    cfsetospeed(&termAttr, baudrate);
    //设置无奇偶校验 8位
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;

    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | ISTRIP);
    termAttr.c_oflag &= ~OPOST;
    termAttr.c_cc[VMIN] = 1;
    termAttr.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &termAttr);
    tcflush(fd, TCIOFLUSH);
    std::cout << port << ": serial init ok！\n";
}

extern atomic_bool ppsTalkOk;
extern atomic_bool ppsReadyFlag;
extern int initXPos, initYPos;
void ReceiveData(int fd)
{
    union
    {
        uint8_t dataC[GET_DATA_LEN];
        float dataF[GET_DATA_LEN / 4];
    } rData;
    struct pollfd fdset[2];
    int ret;
    fdset[0].fd = fd;
    int rc, timeOut = 1;
    fdset[0].events = POLLIN;
    tcflush(fd, TCIOFLUSH);
    int count = 0;
    uint8_t dataCnt = 0;
    int nread;
    char ch;
    char lastCh;
    PPSData_t ppsData;
    PPSData_t correctData;
    PPSData_t lastPPSData;
    ppsOut.open("ppsOut.txt");
    robot.serialFlag = true;
    struct timespec pps_time;
    struct timespec now;
    struct timespec diff_time;
    float ppsInitAng = 150;
    float ppsRerAngle = -60;
    float commDelay = (GET_DATA_LEN + 4) * 10 / 115200;
    runFlag = true;
    tcflush(fd, TCIOFLUSH);
    while (runFlag)
    {
        clock_gettime(CLOCK_REALTIME, &now);
        diff_time.tv_nsec = now.tv_nsec - pps_time.tv_nsec;
        diff_time.tv_sec = now.tv_sec - pps_time.tv_sec;
        int diff_msec = diff_time.tv_sec * 1000 + diff_time.tv_nsec / 1000000;
        if (diff_msec > 200 && ppsReadyFlag)
        {
            robot.serialFlag = false;
        }
        // cout << "serial thread normal" << endl;
        // cout<<ch<<endl;
        while ((nread = read(fd, &ch, 1)) > 0)
        {
            clock_gettime(CLOCK_REALTIME, &pps_time);
            robot.serialFlag = true;
            // if(!ppsTalkOk)
            // cout << "ch: " << ch<<endl;
            switch (count)
            {
            case 0:
                if (ch == 'A' || ch == 'O')
                {
                    count++;
                    lastCh = ch;
                }
                else
                    count = 0;
                break;

            case 1:
                if (ch == 'T')
                {
                    dataCnt = 0;
                    count++;
                    // cout<<"ATok"<<endl;
                }
                else if (ch == 'K' || ch == 'A' || ch == 'X' || ch == 'Y')
                {
                    ppsTalkOk = true;
                    count = 0;
                    cout << "OK" << endl;
                    // 设置X、Y的初始坐标值
                    if (lastCh == 'O' && ch == 'X')
                    {
                        ppsSetX = true;
                        ppsOut << lastCh << ch << endl;
                    }
                    if (lastCh == 'O' && ch == 'Y')
                    {
                        ppsSetY = true;
                        ppsOut << lastCh << ch << endl;
                    }
                    if (lastCh == 'O' && ch == 'A')
                    {
                        ppsSetA = true;
                        ppsOut << lastCh << ch << endl;
                    }
                    if(lastCh == 'O' && ch == 'R')
                    {
                        ppsSetR = true;
                        ppsOut << lastCh << ch << endl;
                    }
                    // OX OY 就清空缓冲区字符，防止上一次数据干扰
                    if (lastCh == 'O' && (ch == 'X' || ch == 'Y' || ch == 'A'))
                    {
                        tcflush(fd, TCIFLUSH);
                    }
                }
                else
                    count = 0;
                break;

            case 2:
                rData.dataC[dataCnt] = ch;
                dataCnt++;
                if (dataCnt >= GET_DATA_LEN)
                {
                    dataCnt = 0;
                    count++;
                }
                break;

            case 3:
                if (ch == 't')
                    count++;
                else
                    count = 0;
                break;

            case 4:
            {
                count = 0;
                if (ch == 'a')
                {
                    // cout<<rData.dataF[0]<<endl;
                    //一帧数据接受完成
                    //cout << rData.dataF[0] << endl;
                    if ((!ppsReadyFlag) && (fabs(rData.dataF[0]) > 0))
                    // if ((!ppsReadyFlag) && (fabs(rData.dataF[0]) > 0))
                    {
                        ppsReadyFlag = true;
                    }
                    if (ppsReadyFlag)
                    {
                        // cout << "ppsSrcData: ";
                        // for (int i = 0; i < 5; i++)
                        // {
                        //     cout << rData.dataF[i] << ' ';
                        // }
                        // cout << endl;
                        // 雷达初始化时对定位数据的处理 InitialFlag
                        if (InitPosFlag )
                        {
                            //将坐标加上初始化的坐标和补偿通信延迟造成的坐标误差
                            ppsData.x = INITXPOS;
                            ppsData.y = INITYPOS;
                            if (ppsSetA)
                            {
                                ppsData.theta = rData.dataF[0];
                                cout<<"dadad"<<endl;
                            }
                            else
                            {
                                ppsData.theta = INITAPOS;
                            }
                            robot.rbotVel = Pos_t(ppsData.speedX, ppsData.speedY, ppsData.angleW);
                            correctData.theta = 0;
                            correctData.x = ppsData.x + robot.ppsCorrect.x;
                            correctData.y = ppsData.y + robot.ppsCorrect.y;

                            InitialCoordinate.x = correctData.x;
                            InitialCoordinate.y = correctData.y;
                            //更新定位系统速度数据
                            pthread_mutex_lock(&mtx); //上锁
                            if (robot.posData.size() == 6)
                            {
                                //删除最前面的定位数据
                                robot.posData.pop_front();
                            }
                            if (robot.posData.size() < 6)
                            {
                                //把初始化坐标值的数据存在最后
                                robot.posData.push_back(Pos_t(ppsData.x, ppsData.y, ppsData.theta));
                            }
                            pthread_mutex_unlock(&mtx); //解锁
                            tcflush(fd, TCIFLUSH);
                            for (int i = 0; i < 5; i++)
                            ppsOut << rData.dataF[i] << ' ';
                            ppsOut << endl;
                        }
                        else
                        {
                            ppsData.angleW = rData.dataF[5];

                            //根据发送字节数补偿收到数据时的实际位姿
                            ppsData.theta = rData.dataF[0] + commDelay * ppsData.angleW;

                            // // 减去旋转带来的速度分量
                            // float tmpX = 0, tmpY = 0;
                            // tmpX = (rData.dataF[1] + ANGTORAD(ppsData.angleW) * sin(ANGTORAD(ppsData.theta)) * PPSTOLIDAR);
                            // tmpY = (rData.dataF[2] - ANGTORAD(ppsData.angleW) * cos(ANGTORAD(ppsData.theta)) * PPSTOLIDAR);
                            // // 将坐标旋转到世界坐标系下
                            // ppsData.speedX = (tmpX * cos(ANGTORAD(ppsInitAng)) - tmpY * sin(ANGTORAD(ppsInitAng)));
                            // ppsData.speedY = (tmpX * sin(ANGTORAD(ppsInitAng)) + tmpY * cos(ANGTORAD(ppsInitAng)));
                            ppsData.speedX = rData.dataF[1];
                            ppsData.speedY = rData.dataF[2];
                            // //平移到车中心的坐标系
                            // tmpX = rData.dataF[3] + PPSTOLIDAR * (1 - cos(ANGTORAD(ppsData.theta)));
                            // tmpY = rData.dataF[4] - PPSTOLIDAR * sin(ANGTORAD(ppsData.theta));
                            // //旋转到与世界坐标系一致
                            // ppsData.x = (tmpX * cos(ANGTORAD(ppsInitAng)) - tmpY * sin(ANGTORAD(ppsInitAng)));
                            // ppsData.y = (tmpX * sin(ANGTORAD(ppsInitAng)) + tmpY * cos(ANGTORAD(ppsInitAng)));
                            ppsData.x = rData.dataF[3];
                            ppsData.y = rData.dataF[4];
                            //将坐标加上初始化的坐标和补偿通信延迟造成的坐标误差
                            // ppsData.x = ppsData.x + INITXPOS + ppsData.speedX * commDelay;
                            // ppsData.y = ppsData.y + INITYPOS + ppsData.speedY * commDelay;
                            ppsData.x = ppsData.x + ppsData.speedX * commDelay;
                            ppsData.y = ppsData.y + ppsData.speedY * commDelay;
                            if ((robot.posData.size() != 0))
                            {
                                JudgePPSData(ppsData, lastPPSData);
                            }
                            lastPPSData.x = ppsData.x;
                            lastPPSData.y = ppsData.y;
                            lastPPSData.theta = ppsData.theta;
                            lastPPSData.speedX = ppsData.speedX;
                            lastPPSData.speedY = ppsData.speedY;
                            lastPPSData.angleW = ppsData.angleW;
                            //更新定位系统速度数据
                            robot.rbotVel = Pos_t(ppsData.speedX, ppsData.speedY, ppsData.angleW);

                            pthread_mutex_lock(&mtxCorrect); //上锁
                            //更新矫正后的定位系统坐标
                            correctData.x = ppsData.x + robot.ppsCorrect.x;
                            correctData.y = ppsData.y + robot.ppsCorrect.y;

                            InitialCoordinate.x = correctData.x;
                            InitialCoordinate.y = correctData.y;
                            //定位系统矫正初始角度后的当前角度值
                            // correctData.theta = ppsData.theta; // 定位的数据直接是车的姿态角
                            correctData.theta = ppsData.theta + ppsInitAng;
                            pthread_mutex_unlock(&mtxCorrect); //解锁
                            LimitAng(correctData.theta);
                            pthread_mutex_lock(&mtx); //上锁

                            ppsInitAng = robot.ppsInitAngle;
                            if (robot.posData.size() == 6)
                            {
                                //删除最前面的定位数据
                                robot.posData.pop_front();
                            }
                            if (robot.posData.size() < 6)
                            {
                                //把最新的定位系统的数据存在最后
                                robot.posData.push_back(Pos_t(ppsData.x, ppsData.y, ppsData.theta));
                            }
                            pthread_mutex_unlock(&mtx); //解锁
                            //将机器人位姿信息写入共享内存
                            SendData2MCU(ppsData, correctData);
                            //ppsOut<<ppsData.speedX<<' '<<ppsData.speedY<<endl;
                            tcflush(fd, TCIFLUSH);
                        }
                    }
                }
                break;
            }
            default:
                count = 0;
                break;
            }
        }
    }
}
/* void SerialSendData(int fd,PPSData_t ppsData,PPSData_t correctData)
{
    union 
    {
        uint8_t dataC[MC_LIDAR_BUF_LENGTH-4];
        float dataF[(MC_LIDAR_BUF_LENGTH-4)/4];
    }sendData;
    
    uint8_t tdata[MC_LIDAR_BUF_LENGTH];
    tdata[0]='A';
    tdata[1]='T';
    tdata[MC_LIDAR_BUF_LENGTH-2]='\r';
    tdata[MC_LIDAR_BUF_LENGTH-1]='\n';

    sendData.dataF[0]=ppsData.x;
    sendData.dataF[1]=ppsData.y;
    sendData.dataF[2]=ppsData.theta;
    sendData.dataF[3]=ppsData.speedX;
    sendData.dataF[4]=ppsData.speedY;
    sendData.dataF[5]=ppsData.angleW;
    sendData.dataF[6]=correctData.x;
    sendData.dataF[7]=correctData.y;
    sendData.dataF[8]=correctData.theta;
    sendData.dataF[6]=oriX;
    sendData.dataF[7]=oriY;
    sendData.dataF[8]=oriAngle;
    memcpy(tdata+2,sendData.dataC,MC_LIDAR_BUF_LENGTH-4);
    write(fd,tdata,MC_LIDAR_BUF_LENGTH);
    // cout<<"发数 "<<ppsData.x<<' '<<ppsData.y<<endl;
} */
void CopyData(uint8_t *origen, uint8_t *afterTreat, int size)
{

    for (size_t i = 0; i < size; i++)
    {
        *afterTreat = *origen;
        afterTreat++;
        origen++;
    }
}
void SendData2MCU(PPSData_t ppsData, PPSData_t correctData)
{
    union
    {
        char dataC[MC_LIDAR_BUF_LENGTH - 4];
        float dataF[(MC_LIDAR_BUF_LENGTH - 5) / 4];
    } sendData;
    static struct timespec last_spec;
    char tdata[MC_LIDAR_BUF_LENGTH];
    tdata[0] = 'A';
    tdata[1] = 'T';
    tdata[MC_LIDAR_BUF_LENGTH - 3] = correctionFlag + 48;
    tdata[MC_LIDAR_BUF_LENGTH - 2] = 't';
    tdata[MC_LIDAR_BUF_LENGTH - 1] = 'a';

    sendData.dataF[0] = ppsData.x;
    sendData.dataF[1] = ppsData.y;
    sendData.dataF[2] = ppsData.theta;
    LimitAng(sendData.dataF[2]);
    sendData.dataF[3] = ppsData.speedX;
    sendData.dataF[4] = ppsData.speedY;
    sendData.dataF[5] = ppsData.angleW;
    sendData.dataF[6] = correctData.x;
    sendData.dataF[7] = correctData.y;
    sendData.dataF[8] = correctData.theta;
    memcpy(tdata + 2, sendData.dataC, MC_LIDAR_BUF_LENGTH - 5);
    struct timespec current_spec;
    clock_gettime(CLOCK_REALTIME, &current_spec);
    long msec = (long)((current_spec.tv_sec - last_spec.tv_sec) * 1000 + (current_spec.tv_nsec - last_spec.tv_nsec) / 1000000);
    // cout<<proStamp<<' '<<msec<<' '<<sendData.dataF[0]<<' '<<sendData.dataF[1]<<' '<<sendData.dataF[2]<<' '
    // <<sendData.dataF[3]<<' '<<sendData.dataF[4]<<' '<<sendData.dataF[5]<<endl;
    clock_gettime(CLOCK_REALTIME, &last_spec);
    ppsOut << proStamp << ' ' << msec << ' ' << sendData.dataF[0] << ' ' << sendData.dataF[1] << ' ' << sendData.dataF[2] << ' '<<sendData.dataF[3]
           << " "<<sendData.dataF[4]<<" "<<sendData.dataF[6] << ' ' << sendData.dataF[7] << ' ' << sendData.dataF[8] << ' ' << correctionFlag << endl;
    // cout << "wait ProcessCommWrite done" << endl;
    // 读取复位信息
    // int str = ReadMCData();
    // if (str == 2)
    // {
    //     tdata[MC_LIDAR_BUF_LENGTH - 3] = '2';
    //     cout << "发送重试标志位" << endl;
    // }
    CorrectAngleAgain();
    //进程通讯发送信息
    ProcessCommWrite(tdata, MC_LIDAR_BUF_LENGTH);
    // cout << "ProcessCommWrite done" << endl;
}
void JudgePPSData(PPSData_t &ppsData, PPSData_t &lastPPSData)
{
    // if (fabs(ppsData.x - lastPPSData.x) > 100)
    // {
    //     ppsData.x = lastPPSData.x;
    // }
    // if (fabs(ppsData.y - lastPPSData.y) > 100)
    // {
    //     ppsData.y = lastPPSData.y;
    // }
    // if (fabs(sqrt(pow(ppsData.speedX, 2) + pow(ppsData.speedY, 2)) - sqrt(pow(lastPPSData.speedX, 2) + pow(lastPPSData.speedY, 2))) > 150)
    if(fabs(ppsData.speedX - lastPPSData.speedX)>1000 || fabs(ppsData.speedY - lastPPSData.speedY)>1000)
    {
        ppsData.speedX = lastPPSData.speedX;
        ppsData.speedY = lastPPSData.speedY;
        ppsData.angleW = lastPPSData.angleW;
        ppsData.theta = lastPPSData.theta;
        ppsData.y = lastPPSData.y;
        ppsData.x = lastPPSData.x;
    }
    // float tmpAng = ppsData.theta - lastPPSData.theta;
    // if (ppsData.angleW > 250)
    // {
    //     ppsData.angleW = lastPPSData.angleW;
    // }
    // if (fabs(fabs(ppsData.angleW * 0.005) - fabs(LimitAng(tmpAng))) > 1)
    // {
    //     ppsData.theta = lastPPSData.theta;
    // }
}
int CorrectAngleAgain()
{
    int retFlag;
    char tdata[MC_LIDAR_BUF_LENGTH];
    ProcessCommRead(tdata, MC_LIDAR_BUF_LENGTH);
    if (tdata[0] == 'S' && tdata[1] == 'T' && tdata[MC_LIDAR_BUF_LENGTH - 2] == '\r' && tdata[MC_LIDAR_BUF_LENGTH - 1] == '\n')
    {
        retFlag = (tdata[MC_LIDAR_BUF_LENGTH - 3]);
        output<<"retFlag "<< " "<<retFlag<<" " <<(int)tdata[MC_LIDAR_BUF_LENGTH - 3]<<endl;
    }
    if(retFlag==3)
    {
        robot.mode = CORRECTANGLE;
        CorrectionAgain = false;
    }
}
int SelfTest()
{
    int retFlag;
    char tdata[MC_LIDAR_BUF_LENGTH];
    ProcessCommRead(tdata, MC_LIDAR_BUF_LENGTH);
    if (tdata[0] == 'S' && tdata[1] == 'T' && tdata[MC_LIDAR_BUF_LENGTH - 2] == '\r' && tdata[MC_LIDAR_BUF_LENGTH - 1] == '\n')
    {
        retFlag = (tdata[MC_LIDAR_BUF_LENGTH - 3]);
        output<<"retFlag2 "<< " "<<retFlag<<" " <<(int)tdata[MC_LIDAR_BUF_LENGTH - 3]<<endl;
    }
    if(retFlag==4)
    {
        correctionFlag = true;
        return 0;

    }
    return 1;

}
// int ReadMCData()
// {
//     int retFlag;
//     char tdata[MC_LIDAR_BUF_LENGTH];
//     ProcessCommRead(tdata, MC_LIDAR_BUF_LENGTH);
//     if (tdata[0] == 'S' && tdata[1] == 'T' && tdata[MC_LIDAR_BUF_LENGTH - 2] == '\r' && tdata[MC_LIDAR_BUF_LENGTH - 1] == '\n')
//     {
//         retFlag = (tdata[MC_LIDAR_BUF_LENGTH - 3] - 48);
//         correctFlag = 0;
//     }
//      if(retFlag==2)
//     {
//         while(!ppsSetR)
//         {
//             uint8_t tdata[4];
//             tdata[0] = 'A';
//             tdata[1] = 'K';
//             tdata[2] = '\r';
//             tdata[3] = '\n';
//             write(ppsSerial, tdata, 4);
//         }
//         // tcflush(fd, TCIFLUSH);
//         ppsSetA = !ppsSetA; 
//         ppsSetX = !ppsSetX; 
//         ppsSetY = !ppsSetY;
//         ppsSetR = !ppsSetR; 
//         correctPosFlag = !correctPosFlag;
//         InitialFlag = !InitialFlag;
//         InitPosFlag = !InitPosFlag;
//         correctionflag = 0;
//         robot.mode = CORRECTANGLE;
//     }
//     return 0;
// }