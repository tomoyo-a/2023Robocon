#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h> 
#include "mcucomm.h"
#include "spi_init.h"
#include "moveBase.h"
#include "robot.h"
#include "timer.h"
#include "pps.h"
#include "ringbuffer.h"

#include "ControlSPI.h"
#include "ErrorType.h"
extern FILE *fpWrite;
extern KeyPointInf_t *ringBuffer;
typedef union 
{
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transData_t;
//修改接收发送时要修改两处接收函数和一处发送函数
//向 mcu master发送消息  
uint8_t tx[MCU_SPI_LENGTH] = {0}; 
//接受 mcu master 的消息
uint8_t rx[MCU_SPI_LENGTH] = {0};

//去除起止符的消息
uint8_t txTransData[MCU_SPI_LENGTH - 5] = {0};
uint8_t rxTransData[MCU_SPI_LENGTH - 5] = {0};
//接收到用不到的数据时用blank接受
float blankFloat[8] = {0};
uint8_t blank7;
//检测SPI函数时间
struct timeval SPIstart, SPIendTime;
long long SPItotal_time,SPIstartTime;


static uint8_t mcuTalkOk = 0;
pthread_t spi_thread_id;
void WaitMcuPrepare(void)
{
    while(!mcuTalkOk);
    // while(!mcuTalkOk)
    // {
    //     usleep(5000);
    //     Communicate2Mcu();
    // }
}

void Communicate2Mcu(void)
{
    uint8_t spiShiftFlag = 0;
    uint8_t SPIIdentifierCorrect = 0; 
    uint8_t predictNum = gRobot.planData.NowposIndex;
    float percent = FIRST_PATH_PERCENT;
    static uint8_t cnt= 0;//
    static uint8_t caculateCnt = 0;
    static uint8_t lastMomentTime =  210;

    // if (gRobot.walkStatus == goForRedPut1st && GetCorrectY() < 3000.f)
    // {
    //     gRobot.bucketColor[0] = WHITE;
    //     gRobot.bucketColor[1] = BLUE;
    //     gRobot.bucketColor[2] = GREEN;
    //     gRobot.bucketColor[3] = BLACK;
    //     gRobot.bucketColor[4] = PINK;
    // }
    // else if (gRobot.walkStatus == goForBluePut1st && GetCorrectY() < 3000.f)
    // {
    //     gRobot.bucketColor[0] = BLACK;
    //     gRobot.bucketColor[1] = BLUE;
    //     gRobot.bucketColor[2] = GREEN;
    //     gRobot.bucketColor[3] = WHITE;
    //     gRobot.bucketColor[4] = PINK;
    // }
    if (predictNum >= gRobot.planData.totalNum - 1)
    {
        predictNum = gRobot.planData.totalNum - 1;
        gRobot.planData.judgeHalfPoint = 0;
    }
    if (gRobot.planData.totalNum == 0)
    {
        predictNum = 0;
        gRobot.planData.judgeHalfPoint = 0;
    }
    // gRobot.preDictVel = ringBuffer[predictNum].vellMax * percent;
    // gRobot.preDictVelDirec = ringBuffer[predictNum].angle;
    // if (gRobot.planData.judgeHalfPoint == 1 && predictNum < gRobot.planData.totalNum - 1)
    // {
    //     gRobot.preDictVel = (ringBuffer[predictNum + 1].vellMax + ringBuffer[predictNum].vellMax)/2.f * percent;
    //     gRobot.preDictVelDirec = (ringBuffer[predictNum + 1].angle + ringBuffer[predictNum].angle)/2.f;
    // }
    // if (gRobot.momentumTime % 5 == 0 && caculateCnt == 0)//60ms更新一次
    if (gRobot.momentumTime != lastMomentTime)
    {
        lastMomentTime = gRobot.momentumTime;
        caculateCnt = 1;
        gRobot.predictChange++;
        if (gRobot.predictChange > 200)
        {
            gRobot.predictChange = 0;
        }
        gRobot.preDictVel =  gRobot.debugInfomation.predictVel;
        gRobot.preDictVelDirec = gRobot.debugInfomation.predictPos.direction;
        gRobot.preDictPoseAngle = ppsPos.angle + gRobot.omg * 30.f / 1000.f * 1.3f;
    }
    // else if (gRobot.momentumTime % 5 != 0)
    {
        caculateCnt = 0;
    }
    //以下为特殊处理，优先级最高
    if (ringBuffer[predictNum].angle > 360.f)
    {
        gRobot.preDictVel = 0.f;
        gRobot.preDictVelDirec = 0.f;
    }
    // if (gRobot.walkStatus == waitForBlueTakingBall)
    // {
    //     gRobot.preDictVel = 220.f;
    //     gRobot.preDictVelDirec = -3.f;
    // }
    // else if (gRobot.walkStatus == waitForBluePut1st )
    // {
    //     gRobot.preDictVel = 125.f;
    //     gRobot.preDictVelDirec = -110.f;
    // }
    if(cnt < 30)
    {
        cnt ++;
        gRobot.robotVel = 0.f;
    }
    // printf("spi2\n");
    // else if (gRobot.walkStatus == goForBlueTakingBall)
    // {
    //     gRobot.preDictVel = 0.f;
    //     gRobot.preDictVelDirec = 0.f;
    // }
    // fprintf(fpWrite,"vel %f %f ",ringBuffer[predictNum].vellMax,ringBuffer[predictNum].angle);
    int read_data_num  = 0;
    int32_t readRet;
    //一直检测是否收到数据
    readRet = VSI_SlaveReadBytes(VSI_USBSPI, 0, rx, &read_data_num, 100);
    // printf("re %d\n", read_data_num);
    //  gRobot.V3AccCaculate = 99.f;
    if (readRet != ERR_SUCCESS)
    {   
        printf("Slave Read data error!!!\n");
    }
    else
    {
        if (read_data_num > 0) //受到数据
        {
            int writeRet=0;
            tx[0] = 'H';
            tx[1] = 'D';
            tx[MCU_SPI_LENGTH - 3] = '\r';
            tx[MCU_SPI_LENGTH - 2] = '\n';
            tx[MCU_SPI_LENGTH - 1] = 0;
            gRobot.cvMcuDir = (int)(gRobot.cvDir * 10);
            WriteSpiData(MCU_SPI_UINT8_CNT,MCU_SPI_FLOAT_CNT,MCU_SPI_LENGTH-5,txTransData,\
                        gRobot.walkStatus, gRobot.putBallFlag,\
                        gRobot.predictChange, gRobot.cvData.filed,\
                        gRobot.bucketColor[0], gRobot.bucketColor[1], gRobot.bucketColor[2],\
                        gRobot.bucketColor[3], gRobot.bucketColor[4],\

                        gRobot.cvData.rectifyErr.x, gRobot.cvData.rectifyErr.y, gRobot.V3AccCaculate,\
                        blankFloat[0], 500.f, gRobot.preDictPoseAngle,\
                        gRobot.preDictVel, gRobot.preDictVelDirec,gRobot.robotVel,\
                        gRobot.velDir,gRobot.omg, blankFloat[0]);
            for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
            {
                tx[i+2] = txTransData[i];
            }
            writeRet = VSI_SlaveWriteBytes(VSI_USBSPI, 0, tx, MCU_SPI_LENGTH);
            if (writeRet != ERR_SUCCESS)
            {
                printf("Slave write data error!!!\n");
            }
            if(rx[0] == 'H' && rx[1] == 'M' && rx[MCU_SPI_LENGTH-3] == '\r' && rx[MCU_SPI_LENGTH-2] == '\n')
            {
                spiShiftFlag  = 0;
                for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
                {
                    rxTransData[i] = rx[i+2];
                }
                SPIIdentifierCorrect = 1; 
            }
            else if(rx[1] == 'H' && rx[2] == 'M' && rx[MCU_SPI_LENGTH-2] == '\r' && rx[MCU_SPI_LENGTH-1] == '\n')
            {
                spiShiftFlag  = 1;
                for(int i = 0; i < (MCU_SPI_LENGTH - 5); i ++)
                {
                    rxTransData[i] = rx[i+3];
                }
                SPIIdentifierCorrect = 1; 
            }
            if(SPIIdentifierCorrect == 1)
            {
                gRobot.mcuHeart = 0;
                mcuTalkOk = 1;//收到数据，mcu通过
                ReadSpiData(MCU_SPI_UINT8_CNT,MCU_SPI_FLOAT_CNT,MCU_SPI_LENGTH-5,rxTransData,\
                        &gRobot.ppsCnt, &gRobot.mcuCnt, &gRobot.gamestart,\
                        &gRobot.cvData.filed, &gRobot.putBallOverFlag, &gRobot.ballNum, &blank7, &blank7, &blank7,\

                        &ppsPos.x, &ppsPos.y, &ppsPos.angle, &ppsPos.speedX,\
                        &ppsPos.speedY, &ppsPos.WZ, &ppsPos.pitchAngle, &ppsPos.pitchSpeed,\
                        &ppsPos.correctX,&ppsPos.correctY,&gRobot.stickLPos, &gRobot.stickRPos);
                SetX(ppsPos.x);
                SetY(ppsPos.y);
                SetAngle(ppsPos.angle);
                SetWZ(ppsPos.WZ);
                SetSpeedX(ppsPos.speedX);
                SetSpeedY(ppsPos.speedY);
                SetPitchAngle(ppsPos.pitchAngle);
                SetPitchSpeed(ppsPos.pitchSpeed);
                SetCorrectX(ppsPos.correctX);
                SetCorrectY(ppsPos.correctY);
                //计算路程
                if(GetY() > 10.f)
                {
                    CaculatePath();
                }
            }
            printf("r ");
            // printf("p %f y%f x %f y%f", ppsPos.pitchAngle, ppsPos.pitchSpeed, GetPitchAngle(), GetPitchSpeed());
            for(int i = 0; i<3;i++ )
            {
                printf("%d ",(int)rx[i]);
            }
            printf("\n");
	        AccBlanceAngle();
        }
    }
    // //与mcu通信
    // //检测函数时间 
    // gettimeofday(&SPIstart, NULL);

    // SPIDataRW(0,tx,rx,MCU_SPI_LENGTH);
    
    // gettimeofday(&SPIendTime, NULL);  
	// SPItotal_time = (SPIendTime.tv_sec - SPIstart.tv_sec) * 1000000 + (SPIendTime.tv_usec - SPIstart.tv_usec);
}
/**
	* @brief	WriteSpiData 将各个变量存入spi需要给下层板发送的数据内容数组中
  * @note		None
  * @param	uint8DataCnt：u8类型的标志位个数 
	* @param	floatDataCnt：float类型的数据个数
	* @param	dataLength: rx数组长度 必须等于n1+n2*4！！！
	* @param	txData：需要存入的数组首地址
	* @param	...：n1个u8类型变量，n2个float类型变量，注意个数！注意只能是u8和float类型变量！！！
  * @retval	None
  */
void WriteSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* txData, ...)
{
    transData_t spiSendMessage;
    va_list args;
    va_start(args, txData); 
    for(uint8_t i = 0; i < uint8DataCnt; i ++)
    {
        txData[i] = (char)va_arg(args, int);
    }
    for(uint8_t i = 0; i < floatDataCnt; i ++)
    {
        spiSendMessage.dataf = (float)va_arg(args, double);
        for(int j = 0; j < 4; j ++)
        {
            txData[uint8DataCnt + i*4 + j] =  spiSendMessage.data8[j];
        }
    }
    va_end(args);
}

/**
	* @brief	ReadData 将下层板数组内容解析给各个变量
  * @note		None
  * @param	uint8DataCnt：u8类型的标志位个数 
	* @param	floatDataCnt：float类型的数据个数
	* @param	dataLength: rx数组长度 必须等于n1+n2*4！！！
	* @param	rxData：需要解析的数组首地址
	* @param	...：n1个u8类型变量取址，n2个float类型变量取址，注意个数！注意只能是u8和float类型变量的地址！！！
  * @retval	
  */
void ReadSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* rxData, ...)
{
    transData_t spiRecMessage;
    va_list args;
    va_start(args, rxData); 

    for(uint8_t i = 0; i < uint8DataCnt; i ++)
    {
        *va_arg(args, uint8_t*) = rxData[i];
    }
    for(uint8_t i = 0; i < floatDataCnt; i ++)
    { 
        for(int j = 0; j < 4; j ++)
        {
            spiRecMessage.data8[j] = rxData[uint8DataCnt + i*4 + j]; 
        }
        *(va_arg(args,float*)) = (float)spiRecMessage.data32/100.f;
    }
    va_end(args);
}

//新线程读取主机数据
void* SpiRun(void* p_comm)
{
    
    while(1)
    {
        Communicate2Mcu();
    }
    return (void*)0;
}

/** @brief 纬图usb转spi模块 初始化配置 小电脑要和单片机初始化同步
 * @name SPIInit
 * @note ERR_SUCCESS 值为0  返回此值 表示没有错误  返回其他负数 则spi初始化过程中出现了相应错误，具体对应什么错，自查数据手册
 */
int SPIInit(void)   
{
    #ifdef SPI_COM
    int ret;
    VSI_INIT_CONFIG SPI_Config;     //spi适配器结构体变量定义
    VSI_BOARD_INFO BoardInfo;       //spi适配器结构体变量定义
    ret = VSI_ScanDevice(1);        //初始化驱动并扫描当前与电脑连接并正常枚举通过的设备数量 0表示无设备连接
    printf("----VSI_ScanDevice %d\n",ret);    
    fprintf(fpWrite,"----VSI_ScanDevice %d\n",ret);  
    if (ret <= 0)
    {
        printf("No SPI device connect! %d \n",ret);
        //return ret;
    }
    else
    {
        printf("%d SPI device connect! %d \n",ret);
    }
    // Open device
    ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);

    printf("----VSI_OpenDevice %d\n",ret);
    fprintf(fpWrite,"----VSI_OpenDevice %d\n",ret);
    if (ret != ERR_SUCCESS)
    {
        printf("Open SPI device error! \n");        //如果发生错误，参考纬图Ginkgo SUB-SPI适配器接口函数库使用手册
    }
    else
    {
        printf("SPI device found! \n");
    }

    //spi初始化配置
    SPI_Config.ControlMode = 0;     //设置SPI控制模式  为全双工硬件模式
    SPI_Config.MasterMode = 0;      //主从机选择 0是从机 1是主机
    SPI_Config.ClockSpeed = 2250000;        //时钟输出频率比  , 此处选择为2.25M HZ 
    SPI_Config.CPHA = 0;        //时钟相位选择  0-第一个时钟沿采样数据  1-第二个时钟沿采样数据
    SPI_Config.CPOL = 0;        //时钟极性，0-时钟信号在空闲时为低电平  1-时钟信号在空闲时为高电平
    SPI_Config.LSBFirst = 0;    //数据传输方向 0-先发高位  1-先发低bit位数据
    SPI_Config.TranBits = 8;    //数据传输bit宽度，8-数据宽度为8bit  16-数据传输宽度为16bit
    SPI_Config.SelPolarity = 0; //片选输出极性 0-有效片选输出为低电平  1-有效片选输出为高电平    
	printf("wait VSI_InitSPI ...... \n"); 
    ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);      //用来存储spi初始化参数的VSI_INIT_CONFIG 结构体指针    
    printf("----VSI_InitSPI %d\n",ret);
    fprintf(fpWrite,"----VSI_InitSPI %d\n",ret);
    if (ret != ERR_SUCCESS)  
    {
        printf("Initialize SPI device error! \n");

        //return ret;
    }
    else
    {
        printf("Initialize SPI device done! \n");
    }
   #endif
     
}
// int SPIInit(void)
// {
//     int ret;
//     VSI_INIT_CONFIG SPI_Config;
//     VSI_BOARD_INFO BoardInfo;
//     ret = VSI_ScanDevice(1);
//     if (ret <= 0)
//     {
//         printf("No SPI device connect! \n");
//         //return ret;
//     }
//     else
//     {
//         printf("%d SPI device connect! \n",ret);
//     }
//     // Open device
//     ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);

//     printf("VSI_OpenDevice %d\n",ret);
//     if (ret != ERR_SUCCESS)
//     {
//         printf("Open SPI device error! \n");
//         //return ret;
//     }
//     else
//     {
//         printf("SPI device found! \n");
//     }

//     SPI_Config.ControlMode = 0;
//     SPI_Config.MasterMode = 0;
//     SPI_Config.ClockSpeed = 2250000;
//     SPI_Config.CPHA = 0;
//     SPI_Config.CPOL = 0;
//     SPI_Config.LSBFirst = 0;
//     SPI_Config.TranBits = 8;
//     SPI_Config.SelPolarity = 0;
//     ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
//     if (ret != ERR_SUCCESS)
//     {
//         printf("Initialize SPI device error! \n");

//         //return ret;
//     }
//     else
//     {
//         printf("Initialize SPI device done! \n");
//     }
   
     
// }
void NewSpiThread(void)
{
    int i, retp;
    retp = pthread_create(&spi_thread_id, NULL, SpiRun, NULL);
    if (retp != 0)
    {
        printf("new thread establish failed!\n");
    }
    else
    {
        printf("new thread established ! ID is %d \n", retp);
    }
}
