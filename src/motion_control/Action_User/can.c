#include "can.h"
#include <string.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

int socket_can0;
int socket_can1;

void CAN_Init(uint8_t canPort)
{
    switch(canPort)
    {
        case CAN0:
        {
            struct sockaddr_can addr_can0;
            struct ifreq ifr_can0; 
            
            socket_can0 = socket(PF_CAN,SOCK_RAW,CAN_RAW);//创建 SocketCAN 套接字
            strcpy(ifr_can0.ifr_name, "can0" );
            ioctl(socket_can0, SIOCGIFINDEX, &ifr_can0);//指定 can0 设备
            addr_can0.can_family = AF_CAN;
            addr_can0.can_ifindex = ifr_can0.ifr_ifindex;
            bind(socket_can0, (struct sockaddr *)&addr_can0, sizeof(addr_can0)); //将套接字与 can0 绑定

            break;
        }
        case CAN1:
        {
            struct sockaddr_can addr_can1;
            struct ifreq ifr_can1;

            socket_can1 = socket(PF_CAN,SOCK_RAW,CAN_RAW);//创建 SocketCAN 套接字
            strcpy(ifr_can1.ifr_name, "can1" );
            ioctl(socket_can1, SIOCGIFINDEX, &ifr_can1);//指定 can0 设备
            addr_can1.can_family = AF_CAN;
            addr_can1.can_ifindex = ifr_can1.ifr_ifindex;
            bind(socket_can1, (struct sockaddr *)&addr_can1, sizeof(addr_can1)); //将套接字与 can0 绑定

            break;
        }
        default:
            break;
    }
}

/* 

//主线程于can接收线程同步于互斥的信号量
sem_t mainSem_id, canRecvSem_id;
//can接受线程的id create返回值 join返回值
pthread_t canRecvThread_id;
int canRecvThreadCreate_retval;
//can接收数据帧
struct can_frame canRecvFrame = {0};
//can接受线程
void CanRecevThread(void)
{
	while(1)
	{
		int canRecvSem_retval = sem_wait(&canRecvSem_id);
		if(canRecvSem_retval >= 0)//获得信号量才会执行
		{
			printf("canRecvSem success\r\n");

			 int nodeID_can0 = 0;

            int nbytes_can0 = read(socket_can0,&canRecvFrame,sizeof(canRecvFrame));
            nodeID_can0 = canRecvFrame.can_id - DRIVER_CLIENT_BASE_ID;
            
            switch(nodeID_can0)
            {
                case ONE_WHEEL_ID:
                {
                    GetDriverMsg(&oneWheelMsgCanRec,canRecvFrame.data);
                    break;
                }
				case ONE_TURN_ID:
                {
                    GetDriverMsg(&oneTurnMsgCanRec,canRecvFrame.data);
                    break;
                }
				case TWO_WHEEL_ID:
                {
                    GetDriverMsg(&twoWheelMsgCanRec,canRecvFrame.data);
                    break;
                }
				case TWO_TURN_ID:
                {
                    GetDriverMsg(&twoTurnMsgCanRec,canRecvFrame.data);
                    break;
                }
				case THR_WHEEL_ID:
                {
                    GetDriverMsg(&thrWheelMsgCanRec,canRecvFrame.data);
                    break;
                }
				case THREE_TURN_ID:
                {
                    GetDriverMsg(&thrTurnMsgCanRec,canRecvFrame.data);
                    break;
                }
                default:
                    break;
            }
            // printf("2 T %d %lld %lld ",GetTimeCounter(),threadTime,totalTime2);       
            // printf("\r\n");

			sem_post(&mainSem_id);  
		}
	}
}
	//小电脑can通信  暂时弃用 原来在初始化函数中

	//初始化CAN0
	CAN_Init(CAN0);

	//初始化同步互斥作用的信号量
	sem_init(&mainSem_id,0,1);
	sem_init(&canRecvSem_id,0,0);

	//创建线程 返回值为0创建成功
	canRecvThreadCreate_retval = pthread_create(&canRecvThread_id,NULL,(void *) CanRecevThread,NULL);
	
	if(canRecvThreadCreate_retval != 0)
	{
		printf("Create canrecv pthread error ! \r\n");
		return 0;
	}
	*/
