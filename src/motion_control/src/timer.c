#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include "timer.h"
#include "robot.h"

static uint8_t timeFlag = 0;
//声明信号量结构体
userSem_t gSem;

struct timeval start2, end2;

long long total_time2;

extern long long total_t,startTime;
 
void signalHandler(void)
{
	//2*10ms = 20ms
	#define PERIOD_COUNTER (2)

	static uint8_t periodCounter = PERIOD_COUNTER;
	
			gettimeofday(&start2, NULL); 

			//10ms发送一次速度环周期信号量
			gSem.velctrlPeriodSem = VELCTRL_PERIOD_SEM_SEND;
			
			periodCounter--;

			if(periodCounter == 0)
			{
				//20ms发送一次位置环周期信号量
				gSem.periodSem = PERIOD_SEM_SEND;

				periodCounter = PERIOD_COUNTER;

				CountTime();				
			}

			//10ms 周期标志位置1
			timeFlag = 1;

			gettimeofday(&end2, NULL);
			
			total_time2 = (end2.tv_sec - start2.tv_sec) * 1000000 + (end2.tv_usec - start2.tv_usec);
			total_t = end2.tv_sec * 1000000 + end2.tv_usec-startTime;
	
}


void init_sigaction()
{
    struct sigaction act;
          
    act.sa_handler = signalHandler; //设置处理信号的函数
    act.sa_flags  = 0;

    sigemptyset(&act.sa_mask);
    sigaction(SIGALRM, &act, NULL);//时间到发送SIGROF信号
}

void init_time()
{
    struct itimerval val;
         
    val.it_value.tv_sec = 0; //1秒后启用定时器
    val.it_value.tv_usec = INTERVAL;

    val.it_interval = val.it_value; //定时器间隔为1s

    setitimer(ITIMER_REAL, &val, NULL);
}



void TimeInit(void)
{
	init_sigaction();
    init_time();

}

//获取5ms周期标志位
uint8_t GetTimeFlag(void)
{
	if(timeFlag == 1)
	{
		timeFlag=0;
		return 1;
	}
	else
		return 0;
	
}





