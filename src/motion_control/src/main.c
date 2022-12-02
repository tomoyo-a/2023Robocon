// #define __USE_GNU
// #define _GNU_SOURCE
#include <stdio.h>
#include <sched.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h> 
#include <sys/time.h>
#include <unistd.h>
#include <sys/sem.h>
#include <semaphore.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include "includes.h"
#include "spi_init.h"
#include "timer.h"
#include "mcucomm.h"
#include "pps.h"
#include "moveBase.h" 
#include "gpio.h"
#include "task.h"
#include "robot.h" 
#include "driver.h"
#include "can.h"
#include "cv.h"
struct timeval start, endTime;
long long total_time,startTime;
//can接受线程的ijoin返回值
void  * canRecvThreadJoin_retval;

FILE *fpWrite;
char filename[23] = {0};


uint8_t writeOnceFlag = 200;

char* getNowtime(void) 
{
	static char s[9]={0};
    char YMD[15] = {0};
    char HMS[10] = {0};
    time_t current_time;
    struct tm* now_time;

    char *cur_time = (char *)malloc(9*sizeof(char));
    time(&current_time);
    now_time = localtime(&current_time);

    strftime(YMD, sizeof(YMD), "%F ", now_time);
    strftime(HMS, sizeof(HMS), "%T", now_time);
    
    strncat(cur_time, YMD+5, 2);
    strncat(cur_time, YMD+8, 2);
    strncat(cur_time, HMS,2);
    strncat(cur_time, HMS+3,2);

	memcpy(s, cur_time, strlen(cur_time)+1);
    free(cur_time);
    cur_time = NULL;
    return s;
}
int main(void)
{	
	strncpy(filename, "../record/D", 11);
	strncat(filename, getNowtime(), 8);
	strncat(filename, ".txt", 4);
	if((fpWrite = fopen(filename,"w")) == NULL)
	{
		printf("open file failed\n");
	}
	else 
	{
		setbuf(fpWrite,NULL);
		printf("start write %s\n",filename);
	}

	
	gettimeofday(&endTime, NULL);
	startTime = (endTime.tv_sec) * 1000000 + (endTime.tv_usec);
	//初始化任务
	ConfigTask();
	printf("config finish\n");
	while(1)
	{
		//程序周期10ms
		if(GetTimeFlag())
		{
			gettimeofday(&start, NULL);
			printf("walk start\n");
			if( gSem.periodSem )
			{
				//位置环20ms周期信号量清零
				gSem.periodSem = PERIOD_SEM_NONE;
				//执行走形任务
				WalkTask();
			}

			if( gSem.velctrlPeriodSem && gSem.velctrlCmdSem )
			{
				//速度环10ms周期信号量清零
				gSem.velctrlPeriodSem = VELCTRL_PERIOD_SEM_NONE;
			
				//速度环命令信号量减一
				gSem.velctrlCmdSem--;

				if(gSem.velctrlCmdSem <= VELCTRL_CMD_SEM_NONE)
				{
					gSem.velctrlCmdSem = VELCTRL_CMD_SEM_NONE;
				}
				//执行速度环任务
				VelCtrlTask();
				
			}
			#ifdef LIDAR_COM
			Talk2Pps();
			#endif
			
			#ifdef CV_COM
			//与视觉通讯
			Talk2Cv();
			if(GetY() > 0.f)
			{
				SendCVData();
			}
			//cv rectify
			CvRectifyPosPerTime();
			#endif
			#ifdef SPI_COM
			// //与mcu通信
			gRobot.mcuHeart++;
			#endif
			//发送调试数据
			//SendDebugInfo();
			WriteDebugInfo2File();

			gettimeofday(&endTime, NULL);  
			total_time = (endTime.tv_sec - start.tv_sec) * 1000000 + (endTime.tv_usec - start.tv_usec);
		}
	}	
	return (0);
}




















