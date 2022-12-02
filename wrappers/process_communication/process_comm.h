#ifndef PROCESS_COMM_H_
#define PROCESS_COMM_H_

#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <errno.h> 
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <poll.h>
#include <sys/sem.h>//包含信号量定义的头文件 

#define MC_LIDAR_BUF_LENGTH 41   //修改此数值后需重启电脑，否则程序出错                
#define MC_LIDAR_KEY_ID    ((key_t)(2048))
#define MC_CV_BUF_LENGTH 33
#define MC_CV_ID ((key_t)(4096))



#define MC (1)
#define LIDAR (2)
#define CV (3)


int ProcessCommInit(key_t key,u_int8_t deviceType);
int ProcessCommWrite(char* writrData,int length);
int ProcessCommRead(char* recData,int length);
int ProcessCommRelease(int length);

//函数声明
//函数：设置信号量的值
static int set_semvalue(int sem_id);
//函数：删除信号量
static void del_semvalue(int sem_id);
//函数：信号量P操作
static int semaphore_p(int sem_id);
//函数：信号量V操作
static int semaphore_v(int sem_id);

#endif
