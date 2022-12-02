#ifndef _TASK_H
#define _TASK_H

#include "driver.h"
#include "can.h"
#include <sys/sem.h>
#include <semaphore.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>


//主线程于can接收线程同步于互斥的信号量
extern sem_t mainSem_id, canRecvSem_id;

extern pthread_t canRecvThread_id;

void ConfigTask(void);

void VelCtrlTask(void);

void WalkTask(void);

void SendDebugInfo(void);
void WriteDebugInfo2File(void);

void TestTurnWeel(void);

int8_t ThreadBindCPU(void);

void SerialInit(void);

void TestPath(void);

void fun_sig(int sig);

#endif
