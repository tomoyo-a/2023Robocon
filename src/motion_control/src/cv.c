#include "cv.h"
#include "process_comm.h"
#include "pps.h"
#include "mcucomm.h"
#include "robot.h"
#include "balance.h"
 

transCVData_t cvReceiveMessage;
uint8_t CvTalkOk = 0;
char recCvData[MC_CV_BUF_LENGTH] = {0};
//等待cv初始化
void WaitCvPrepare(void)
{
	ProcessCommInit(MC_CV_ID,MC);
    // printf("success 111\n\n\n");
    char cvDataInit[MC_CV_BUF_LENGTH] = {0};
    printf("start write data  ");
    ProcessCommWrite(cvDataInit,MC_CV_BUF_LENGTH);
    // //加入一定的响应机制
    while(!CvTalkOk)
    {
        usleep(5000);
        Talk2Cv();
    //    printf("23\r\n");
    }
}

void Talk2Cv(void) //定周期运行
{
	ProcessCommRead(recCvData, MC_CV_BUF_LENGTH);//读取视觉发来的数据，复制到recData中
    // printf("x y \r\n");
    CvDataRecognize();//指令识别
    
}

//接受视觉信息
void CvDataRecognize(void)
{
    // printf("start %d %d ", recCvData[0], recCvData[1]);
    if(recCvData[0] == 'C' && recCvData[1] == 'V' && recCvData[MC_CV_BUF_LENGTH - 2] == '\r' && recCvData[MC_CV_BUF_LENGTH - 1] == '\n')
    {
        CvTalkOk = 1;
        gRobot.cvData.cvPeriod = recCvData[2]; //视觉周期
        gRobot.cvData.colorFlag = recCvData[3];//颜色标志位
        gRobot.cvData.locateFlag = recCvData[4]; //矫正球/桶的标志位（识别到是1，未识别到是0）
        gRobot.cvData.lBaffleFlag = recCvData[5];//矫正墙的标志位
        gRobot.cvData.colorMsg[0] = recCvData[6];//5个桶的颜色排布
        gRobot.cvData.colorMsg[1] = recCvData[7];//5个桶的颜色排布
        gRobot.cvData.colorMsg[2] = recCvData[8];//5个桶的颜色排布
        gRobot.cvData.colorMsg[3] = recCvData[9];//5个桶的颜色排布
        gRobot.cvData.colorMsg[4] = recCvData[10];//5个桶的颜色排布
        printf("cvChar %d %d %d %d ", gRobot.cvData.cvPeriod, gRobot.cvData.colorMsg[0], gRobot.cvData.locateFlag, gRobot.cvData.lBaffleFlag);


        //接收球或桶的角度
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+11];
        }
        SetBallBucketAngle(cvReceiveMessage.dataf);

        //接收球或桶的距离
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+15];
        }
        SetBallBucketDis(cvReceiveMessage.dataf);

        //接收过道中心角度
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+19];
        }
        SetBaffleAngle(cvReceiveMessage.dataf);

        //接收过道中心或侧墙距离
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+23];
        }
        SetlBaffleDis(cvReceiveMessage.dataf);

        //接收放球区墙的偏航角
        for(uint8_t i = 0; i < 4; i++)
        {
            cvReceiveMessage.data8[i] = recCvData[i+27];
        }
        SetWallYawAngle(cvReceiveMessage.dataf);
        //存储视觉正确的桶的颜色顺序
        if (gRobot.cvData.colorFlag == 1 && (gRobot.walkStatus == goForRedPut1st || gRobot.walkStatus == goForBluePut1st))
        {
            for (int i = 0;i<5;i++)
            {
                gRobot.bucketColor[i] = gRobot.cvData.colorMsg[i];
            }
        }
        // printf("cvFloat %f %f %f %f \r\n", gRobot.cvData.ballBucketDis, gRobot.cvData.ballBucketAngle, gRobot.cvData.lBaffleDis, gRobot.cvData.baffleAngle);
    }     
}

void SetBallBucketAngle(float setValue)
{
	gRobot.cvData.ballBucketAngle = setValue;
}

void SetBallBucketDis(float setValue)
{
	gRobot.cvData.ballBucketDis = setValue;
}

void SetBaffleAngle(float setValue)
{
	gRobot.cvData.baffleAngle = setValue;
}

void SetlBaffleDis(float setValue)
{
    gRobot.cvData.lBaffleDis = setValue;
} 

void SetWallYawAngle(float setValue)
{
    gRobot.cvData.wallYawAngle = setValue;
} 

/**将红蓝场、桶号、x、y、转盘角度传给视觉
  * 
  * 
  *
  */
void SendCVData(void)
{
    // printf("send111111111111111111111111");
    // gRobot.colorFlag = 1;//red
    // gRobot.attackPotID = 5;//2B
    union 
    {
        char dataC[3];
        float dataF[3];
    }sendData;
    
    char tdata[SEND_CV_DATA];
    tdata[0] = 'M';
    tdata[1] = 'C'; 
    tdata[SEND_CV_DATA-2] = '\r';
    tdata[SEND_CV_DATA-1] = '\n';

    tdata[2] = gRobot.cvData.filed; //红蓝场
    tdata[3] = gRobot.cvData.ballOrBucket;//取球or放球
    tdata[4] = gRobot.cvData.baffleOrLeft;//中间的墙or识别侧墙

    sendData.dataF[0]=GetPitchAngle();
    sendData.dataF[1]=GetAngle();
    sendData.dataF[2]=GetX();

    memcpy(tdata+5,sendData.dataC,12);//？
    

    // printf("send start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    //进程通讯发送信息
    ProcessCommWrite(tdata,MC_CV_BUF_LENGTH);
    
}
