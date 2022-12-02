#include "pps.h"
#include "spi_init.h"
#include "moveBase.h"
#include "robot.h"
#include "timer.h"
#define LIDAR_COOR
extern FILE *fpWrite;

transPPSData_t ppsReceiveMessage;

static pps_t ppsPos;
static pps_t realPos;
uint8_t ppsTalkMontionCtrlOk = 0;
char recPpsData[MC_LIDAR_BUF_LENGTH] = {0};
char lastRecPpsData[MC_LIDAR_BUF_LENGTH] = {0};
char ppsDataInit[MC_LIDAR_BUF_LENGTH] = {0};
char writePpsData[MC_LIDAR_BUF_LENGTH] = {0};
//等待定位系统
void WaitPpsPrepare(void)
{
    ProcessCommInit(MC_LIDAR_KEY_ID, MC);

    ProcessCommWrite(ppsDataInit, MC_LIDAR_BUF_LENGTH);
    //加入一定的响应机制
    while (!ppsTalkMontionCtrlOk)
    {
        usleep(5000);
        Talk2Pps();
    }
}

void Talk2Pps(void) //定周期运行
{
    ProcessCommRead(recPpsData, MC_LIDAR_BUF_LENGTH); //读取雷达发来的定位的数据，复制到recData中
    PpsDataRecognize();                               //指令识别，并计算里程
    // 2是他收到了；3是他矫正完了
    gRobot.DRRetryReturn = recPpsData[MC_LIDAR_BUF_LENGTH - 3] - 48;
    //编辑发送的信息
    //重试标志位
    writePpsData[0] = 'S';
    writePpsData[1] = 'T';
    writePpsData[MC_LIDAR_BUF_LENGTH - 3] = 3;
    // printf("retryflag  %d ",(int)writePpsData[2]);
    writePpsData[MC_LIDAR_BUF_LENGTH - 2] = '\r';
    writePpsData[MC_LIDAR_BUF_LENGTH - 1] = '\n';
    // ProcessCommWrite(writePpsData,MC_LIDAR_BUF_LENGTH);
}

void PpsDataRecognize(void)
{
    if (ppsTalkMontionCtrlOk == 1)
    {
        //心跳包自加
        gRobot.ppsHeart++;
        gRobot.ppsSameHeart++;
        if (gRobot.ppsHeart >= 100)
        {
            gRobot.ppsHeart = 100;
        }
        if (gRobot.ppsSameHeart >= 100)
        {
            gRobot.ppsSameHeart = 100;
        }

        static int POSerrCnt = 0;
        if (fabs(GetX()) < 0.5 || fabs(GetY()) < 0.5)
        {
            POSerrCnt++;
            if (POSerrCnt >= 20)
            {
                gRobot.walkStatus = stop;
                fprintf(fpWrite, "POSerr ");
                POSerrCnt = 0;
            }
        }
        else
        {
            POSerrCnt = 0;
        }
    }
    if (recPpsData[0] == 'A' && recPpsData[1] == 'T' && recPpsData[MC_LIDAR_BUF_LENGTH - 2] == 't' && recPpsData[MC_LIDAR_BUF_LENGTH - 1] == 'a')
    {

        // gRobot.ppsHeart = 0;
        for (uint8_t i = 0; i < 4; i++)
        {
            ppsReceiveMessage.data8[i] = recPpsData[i + 2];
        }
        if (fabs(ppsReceiveMessage.dataf) > 0.1f)
        {
            if (recPpsData[MC_LIDAR_BUF_LENGTH - 3] == '1')
            {
                ppsTalkMontionCtrlOk = 1;
            }
#ifdef LIDAR_COOR
            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 10];
            }
            SetCorrectAngle(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 2];
            }
            SetCorrectX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 6];
            }
            SetCorrectY(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 14];
            }
            SetSpeedX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 18];
            }
            SetSpeedY(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 22];
            }
            SetWZ(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 34];
            }
            SetAngle(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 26];
            }
            SetX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 30];
            }
            SetY(ppsReceiveMessage.dataf);

#else
            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 10];
            }
            SetAngle(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 2];
            }
            SetX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 6];
            }
            SetY(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 14];
            }
            SetSpeedX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 18];
            }
            SetSpeedY(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 22];
            }
            SetWZ(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 26];
            }
            SetCorrectX(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 30];
            }
            SetCorrectY(ppsReceiveMessage.dataf);

            for (uint8_t i = 0; i < 4; i++)
            {
                ppsReceiveMessage.data8[i] = recPpsData[i + 34];
            }
            SetCorrectAngle(ppsReceiveMessage.dataf);

#endif
        }
        else
        {
            fprintf(fpWrite, "pps blank ");
            for (int i = 0; i < MC_LIDAR_BUF_LENGTH; i++)
            {
                fprintf(fpWrite, "%d ", (int)recPpsData[i]);
            }
            fprintf(fpWrite, "\n");
        }

        if (strcmp(lastRecPpsData, recPpsData) != 0) //不相同,说明通信没问题
        {
            gRobot.ppsSameHeart = 0;
        }
        gRobot.ppsHeart = 0;
        strcpy(lastRecPpsData, recPpsData);

    }
}

void SetX(float setValue)
{
    static float oldPosX = 0.0f;

    ppsPos.x = setValue;

    // TR检测异常横坐标
    if (gRobot.courdID == BLUE_COURT)
    {
        // if(ppsPos.x  > 6500.0f || ppsPos.x < -700.0f)
        // {
        //     printf("POSX ERROR er %d ", (int)ppsPos.x);

        //     ppsPos.x = oldPosX + GetSpeedX()*INTERVAL/1000000.0f;

        //     printf("cr %d \r\n", (int)ppsPos.x);
        // }
    }
    else if (gRobot.courdID == RED_COURT)
    {
        // if(ppsPos.x  < -6500.0f || ppsPos.x > 700.0f)
        // {
        //     printf("POSX ERROR er %d ", (int)ppsPos.x);

        //     ppsPos.x = oldPosX + GetSpeedX()*INTERVAL/1000000.0f;

        //     printf("cr %d \r\n", (int)ppsPos.x);
        // }
    }
    oldPosX = ppsPos.x;
}

void SetY(float setValue)
{
    static float oldPosY = 0.0f;

    ppsPos.y = setValue; // + DIS_OPS2CENTER*sinf(ANGLE2RAD(GetAngle())) + DISY_OPS2CENTER ;
    // realPos.y = ppsPos.x + 135.0f*sin(ANGLE2RAD(ppsPos.angle));

    // TR检测异常纵坐标
    if (ppsPos.y > 10300.0f || ppsPos.y < -700.0f)
    {
        // printf("POSY ERROR er %d ", (int)ppsPos.y);

        // ppsPos.y = oldPosY + GetSpeedY()*INTERVAL/1000000.0f;

        // printf("cr %d \r\n", (int)ppsPos.y);
    }
    oldPosY = ppsPos.y;
}

void SetAngle(float setValue)
{
    static float oldAngle = 0.0f;

    ppsPos.angle = setValue;

    //检测异常角度
    if (fabs(ppsPos.angle) > 270.0f)
    {
        printf("ANGLE ERR er %d ", (int)ppsPos.angle);

        ppsPos.angle = oldAngle;

        printf("cr %d \r\n", (int)ppsPos.angle);
    }
    oldAngle = ppsPos.angle;
}

void SetSpeedX(float setValue)
{
    static float oldSpeedX = 0.0f;

    ppsPos.speedX = setValue;

    //检测异常速度
    if (fabs(ppsPos.speedX) > 10000.0f)
    {
        printf("SPEEDX ERR er %d ", (int)ppsPos.speedX);

        ppsPos.speedX = oldSpeedX;

        printf("cr %d \r\n", (int)ppsPos.speedX);
    }
    oldSpeedX = ppsPos.speedX;
}

void SetSpeedY(float setValue)
{
    static float oldSpeedY = 0.0f;

    ppsPos.speedY = setValue;

    //检测异常速度
    if (fabs(ppsPos.speedY) > 10000.0f)
    {
        printf("SPEEDY ERR er %d ", (int)ppsPos.speedY);

        ppsPos.speedY = oldSpeedY;

        printf("cr %d \r\n", (int)ppsPos.speedY);
    }
    oldSpeedY = ppsPos.speedY;
}

void SetWZ(float setValue)
{
    ppsPos.WZ = setValue;
}

void SetPitchAngle(float setValue)
{
    ppsPos.pitchAngle = setValue;
}

void SetPitchSpeed(float setValue)
{
    ppsPos.pitchSpeed = setValue;
}

float GetX(void)
{
    return ppsPos.x;
}

float GetY(void)
{
    return ppsPos.y;
}

float GetAngle(void)
{
    return ppsPos.angle;
}

float GetSpeedX(void)
{
    return ppsPos.speedX;
}

float GetSpeedY(void)
{
    return ppsPos.speedY;
}

float GetWZ(void)
{
    return ppsPos.WZ;
}

float GetPitchAngle(void)
{
    return ppsPos.pitchAngle;
}

float GetPitchSpeed(void)
{
    return ppsPos.pitchSpeed;
}

void SetCorrectX(float setValue)
{
    ppsPos.correctX = setValue;
}

void SetCorrectY(float setValue)
{
    ppsPos.correctY = setValue;
}

void SetCorrectAngle(float setValue)
{
    ppsPos.correctAngle = setValue;
}

float GetCorrectX(void)
{
    return ppsPos.correctX;
}

float GetCorrectY(void)
{
    return ppsPos.correctY;
}

float GetCorrectAngle(void)
{
    return ppsPos.correctAngle;
}
//返回减去绕机器人中心旋转角速度在定位系统位置产生的线速度后的速度
velVector_t GetSpeedWithoutOmega(void)
{
    velVector_t vel = {0.0f};

    // float rotateVel , rotateVelDirection = 0.0f;

    // rotateVel = ANGLE2RAD(GetWZ())*DIS_OPS2CENTER;

    // // rotateVelDirection = 90.0f + RAD2ANGLE(atan2f(DISY_OPS2CENTER,DISX_OPS2CENTER)) + GetAngle();
    // rotateVelDirection = RAD2ANGLE(atan2f(-DIS_OPS2CENTER,0.0f)) + GetAngle();

    // AngleLimit(&rotateVelDirection);

    // vel.xVel = GetSpeedX() - rotateVel * cosf(ANGLE2RAD(rotateVelDirection));
    // vel.yVel = GetSpeedY() - rotateVel * sinf(ANGLE2RAD(rotateVelDirection));
    vel.xVel = GetSpeedX();
    vel.yVel = GetSpeedY();
    return vel;
}

// void ReceiveData(int fd)
// {
//     union
//     {
//         uint8_t dataC[GET_DATA_LEN];
//         float dataF[GET_DATA_LEN / 4];
//     } rData;
//     struct pollfd fdset[2];
//     int ret;
//     fdset[0].fd = fd;
//     int rc, timeOut = 1;
//     fdset[0].events = POLLIN;
//     tcflush(fd, TCIOFLUSH);
//     int count = 0;
//     uint8_t dataCnt = 0;
//     int nread;
//     char ch;
//     char lastCh;
//     // robot.serialFlag = true;
//     struct timespec pps_time;
//     struct timespec now;
//     struct timespec diff_time;
//     float ppsInitAng = 150;
//     float ppsRerAngle = -60;
//     float commDelay = (GET_DATA_LEN + 4) * 10 / 115200;
//     tcflush(fd, TCIOFLUSH);
//     while (1)
//     {
//         ppsTalkMontionCtrlOk = 0;
//         diff_time.tv_nsec = now.tv_nsec - pps_time.tv_nsec;
//         diff_time.tv_sec = now.tv_sec - pps_time.tv_sec;
//         int diff_msec = diff_time.tv_sec * 1000 + diff_time.tv_nsec / 1000000;
//         if (diff_msec > 200 && ppsReadyFlag)
//         {
//             robot.serialFlag = false;
//         }
//         // cout << "serial thread normal" << endl;
//         // cout<<ch<<endl;
//         while ((nread = read(fd, &ch, 1)) > 0)
//         {
//             // robot.serialFlag = true;
//             // if(!ppsTalkOk)
//             // cout << "ch: " << ch<<endl;
//             switch (count)
//             {
//             case 0:
//                 if (ch == 'A')
//                 {
//                     count++;
//                     lastCh = ch;
//                 }
//                 else
//                     count = 0;
//                 break;

//             case 1:
//                 if (ch == 'T')
//                 {
//                     dataCnt = 0;
//                     count++;
//                     // cout<<"ATok"<<endl;
//                 }
//                 else
//                     count = 0;
//                 break;

//             case 2:
//                 rData.dataC[dataCnt] = ch;
//                 dataCnt++;
//                 if (dataCnt >= GET_DATA_LEN)
//                 {
//                     dataCnt = 0;
//                     count++;
//                 }
//                 break;

//             case 3:
//                 if (ch == 't')
//                     count++;
//                 else
//                     count = 0;
//                 break;

//             case 4:
//             {
//                 count = 0;
//                 if (ch == 'a')
//                 {
//                     ppsTalkMontionCtrlOk = 1;

//                     SetSpeedX(rData.dataF[1]);
//                     SetSpeedY(rData.dataF[2]);
//                     SetX(rData.dataF[3]);
//                     SetY(rData.dataF[4]);
//                     SetAngle(rData.dataF[5]);
//                 }
//                 break;
//             }
//             default:
//                 count = 0;
//                 break;
//             }
//         }
//         if(ppsTalkMontionCtrlOk == 1)
//         {
//             break;
//         }
//     }
// }
