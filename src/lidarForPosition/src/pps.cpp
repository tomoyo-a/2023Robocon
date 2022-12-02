#include "serial.h"
#include "pps.h"
#include <ctime>
#include "time.h"
atomic_bool ppsTalkOk;
atomic_bool ppsReadyFlag;
/*ms级延时*/
void Delay_ms(int time)
{
    struct timespec start_spec;
    struct timespec current_spec;
    int msec_time = 0;
    clock_gettime(CLOCK_REALTIME, &start_spec);
    //CLOCK计数单位是us
    while (msec_time < time)
    {
        clock_gettime(CLOCK_REALTIME, &current_spec);
        msec_time =
            (current_spec.tv_sec - start_spec.tv_sec) * 1000 + (current_spec.tv_nsec - start_spec.tv_nsec) / 1000000;
    };
}

/*测试与定位系统通信是否正常
  给定位系统发送 AT 返回OK
  @param fd 发送时使用的串口号
*/
void TalkToPps(int fd)
{
    uint8_t tdata[4];
    tdata[0] = 'A';
    tdata[1] = 'T';
    tdata[2] = '\r';
    tdata[3] = '\n';

    ppsTalkOk = false;
    while (!ppsTalkOk)
    {
        
        Delay_ms(2);
        // cout << "wait pps return ok" << endl;
        write(fd, tdata, 4);
    }
    std::cout << "pps connect ok" << std::endl;
}
/*等待定位系统初始化*/
void WaitPpsReady(int fd)
{
    // TalkToPps(fd);
    ppsReadyFlag = false;
    cout << "Wait pps init ......" << endl;
    while (!ppsReadyFlag)
    {
    }
    
}

/*矫正定位系统角度
 *@param fd 与定位系统通信所用串口号
 *@param type 矫正对象
 *@param setValue 矫正的目标值
 */
void CorrectPps(int fd, char type, float setValue)
{
    union
    {
        uint8_t dataC[4];
        float dataF;
    } sendData;

    uint8_t tdata[8];
    tdata[0] = 'A';
    tdata[6] = '\r';
    tdata[7] = '\n';
    switch (type)
    {
    //矫正X
    case SETX:
        tdata[1] = 'X';
        break;
    //矫正Y
    case SETY:
        tdata[1] = 'Y';
        break;
    //矫正角度
    case SETA:
        tdata[1] = 'A';
        break;
    default:
        //退出函数
        return;
    }
    sendData.dataF = setValue;
    memcpy(tdata + 2, sendData.dataC, 4);
    ppsTalkOk = 0;
    while (!ppsTalkOk)
    {
        Delay_ms(1);
        write(fd, tdata, 8);
    }
}
