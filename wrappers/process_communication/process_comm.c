#include "process_comm.h"

//联合类型semun定义
union semun{
	int val;
	struct semid_ds *buf;
	unsigned short *array;
};
static int pps_sem_id,cv_sem_id;//信号量ID

//共享内存标识符
int shmidPps , shmidCv;
//指向共享内存第一个字节的指针
char *pps_shm_addr, *cv_shm_addr;

int ProcessCommInit(key_t key, u_int8_t deviceType)
{ 
    if(key == MC_LIDAR_KEY_ID)
    {
        // 使用约定的键值创建共享内存
        shmidPps = shmget((key_t)key,  MC_LIDAR_BUF_LENGTH, 0666 | IPC_CREAT);
        printf("pps shmid : %u\n", shmidPps);
        if (shmidPps == -1)
        {
            printf("pps create share memory failed!!!\n\n\n");
            return 1;
        }
        //将共享内存附加到本进程
        //申请共享内存指针
        pps_shm_addr = (char*)shmat(shmidPps, NULL, 0);
        if (pps_shm_addr == NULL)
        {
            printf("pps get share pointer failed!!!\n\n\n");
            return 1;
        }
        // 使用约定的键值创建信号量
        pps_sem_id = semget((key_t)(key+1024),1,0666 | IPC_CREAT);
        if(deviceType == MC)//
        {
            if(!set_semvalue(pps_sem_id))
            {
                printf("failed to initialize semaphore!!!\n\n\n");
                return 1;
            }
        }
    }
    else if(key == MC_CV_ID)
    {
        // 使用约定的键值创建共享内存
        shmidCv = shmget((key_t)key,  MC_CV_BUF_LENGTH, 0666 | IPC_CREAT);
        printf("cv shmid : %u\n", shmidCv);
        if (shmidCv == -1)
        {
            printf("cv create share memory failed!!!\n\n\n");
            return 1;
        }
        
        //将共享内存附加到本进程
        //申请共享内存指针
        cv_shm_addr = (char*)shmat(shmidCv, NULL, 0);
        if (cv_shm_addr == NULL)
        {
            printf("cv get share pointer failed!!!\n\n\n");
            return 1;
        }    

            // 使用约定的键值创建信号量
        cv_sem_id = semget((key_t)(key+1024),1,0666 | IPC_CREAT);
        if(deviceType == MC)//
        {
            if(!set_semvalue(cv_sem_id))
            {
                printf("failed to initialize semaphore!!!\n\n\n");
                return 1;
            }
        }
        //printf("cv \n");
    }
    return 0;
}

int ProcessCommWrite(char* writeData,int length)
{
    if(length == MC_LIDAR_BUF_LENGTH)//commiut with lidar
    {
        printf("try2write");
        if(!semaphore_p(pps_sem_id))
        {
            printf("write busy\n");
            return 0;
        }
        printf("wrote");
        memcpy((pps_shm_addr), writeData, length);
        for(int i = 0;i<3;i++)
        {
            printf("%d ",(int)writeData[i]);
        }
        printf("\n");
        if(!semaphore_v(pps_sem_id))
        {
            return 1;
        }
    }
    else if(length == MC_CV_BUF_LENGTH)//commiut with lidar
    {
        //printf("try2write");
        if(!semaphore_p(cv_sem_id))
        {
            printf("write busy\n");
            return 0;
        }
        // printf("cvwrote");
        memcpy((cv_shm_addr), writeData, length);
        // for(int i = 0;i<MC_CV_BUF_LENGTH;i++)
        // {
        //     printf("%d ",(int)writeData[i]);
        // }
        // printf("\n");
        if(!semaphore_v(cv_sem_id))
        {
            return 1;
        }
    }
}

int ProcessCommRead(char* recData,int length)
{
    if(length == MC_LIDAR_BUF_LENGTH)//commiut with lidar
    {
        // printf("try2read");
        if(!semaphore_p(pps_sem_id))
        {
            printf("liread busy\n");
            return 0;
        }
        printf("liread");
        memcpy(recData, (pps_shm_addr), MC_LIDAR_BUF_LENGTH);
        for(int i = 0;i<3;i++)
        {
            printf("%d ",(int)recData[i]);
        }
        printf("\n");   
        if(!semaphore_v(pps_sem_id))
        {
            return 1;
        }
    }
    else if(length == MC_CV_BUF_LENGTH)//commiut with lidar
    {
        //printf("try2read");
        if(!semaphore_p(cv_sem_id))
        {
            printf("read busy\n");
            return 0;
        }
        // printf("cvread");
        memcpy(recData, (cv_shm_addr), length);
        // for(int i = 0;i<MC_CV_BUF_LENGTH;i++)
        // {
        //     printf("%d ",(int)recData[i]);
        // }
        // printf("\n"); 
        if(!semaphore_v(cv_sem_id))
        {
            return 1;
        }
    }
}

int ProcessCommRelease(int length)
{
    if(length == MC_LIDAR_BUF_LENGTH)//commiut with lidar
    {
        // 分离
        if (shmdt(pps_shm_addr) == -1)
        {
            printf("shmdt error!\n");
            return -1;
        }
        // 删除共享内存
        if (shmctl(shmidPps, IPC_RMID, 0) == -1)
        {
            printf("shmctl error!\n");
            return -1;
        }
    }
    else if(length == MC_CV_BUF_LENGTH)//commiut with lidar
    {
        // 分离
        if (shmdt(cv_shm_addr) == -1)
        {
            printf("shmdt error!\n");
            return -1;
        }
        // 删除共享内存
        if (shmctl(shmidCv, IPC_RMID, 0) == -1)
        {
            printf("shmctl error!\n");
            return -1;
        }
    }
    return 0;
}

//函数：设置信号量的值
static int set_semvalue(int sem_id)
{
	union semun sem_union;
	sem_union.val = 1;
 
	if(semctl(sem_id,0,SETVAL,sem_union) == -1)
    {
        return 0;
    }
 
	return 1;
}
 
//函数：删除信号量
static void del_semvalue(int sem_id)
{
	union semun sem_union;
 
	if(semctl(sem_id,0,IPC_RMID,sem_union) == -1)
		fprintf(stderr,"Failed to delete semaphore\n");
}
 
//函数：信号量P操作：对信号量进行减一操作  
static int semaphore_p(int sem_id)
{
	struct sembuf sem_b;
 
	sem_b.sem_num = 0;//信号量编号
	sem_b.sem_op = -1;//P操作	
	sem_b.sem_flg = SEM_UNDO;
 
	if(semop(sem_id,&sem_b,1) == -1)
	{
	    //fprintf(stderr,"semaphore_p failed\n");
	    return 0;
	}
 
	return 1;
}
 
//函数：信号量V操作：对信号量进行加一操作
static int semaphore_v(int sem_id)
{
	struct sembuf sem_b;
 
	sem_b.sem_num = 0;//信号量编号
	sem_b.sem_op = 1;//V操作	
	sem_b.sem_flg = SEM_UNDO;
 
	if(semop(sem_id,&sem_b,1) == -1)
	{
	    fprintf(stderr,"semaphore_v failed\n");
	    return 0;
	}
 
	return 1;
 
}
