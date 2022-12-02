#ifndef __TIME_H
#define __TIME_H

#include <stdint.h>

//间隔时间，单位是us
#define INTERVAL 5000 

//复位信号量宏定义
#define RESET_SEM_SEND (1)
#define RESET_SEM_NONE (0)
  
//位置环10ms周期信号量宏定义
#define PERIOD_SEM_SEND (1)
#define PERIOD_SEM_NONE (0) 

//速度环5ms周期信号量宏定义
#define VELCTRL_PERIOD_SEM_SEND (1)
#define VELCTRL_PERIOD_SEM_NONE (0)

//速度环命令宏定义
#define VELCTRL_CMD_SEM_SEND (2)
#define VELCTRL_CMD_SEM_NONE (0)

//信号量结构体
typedef struct
{
    uint8_t periodSem;
    uint8_t velctrlPeriodSem;
    uint8_t velctrlCmdSem;
}userSem_t;

//声明信号量结构体
extern userSem_t gSem;

void signalHandler(void);

void TimeInit(void);

//获取10ms周期标志位
uint8_t GetTimeFlag(void);

#endif 
