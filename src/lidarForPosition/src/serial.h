#ifndef SERIAL_H
#define SERIAL_H
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <poll.h>
#include <time.h>
//#include "data.h"
#include "pos.h"

void CopyData(uint8_t *origen, uint8_t *afterTreat, int size);
void JudgePPSData(PPSData_t &ppsData, PPSData_t &lastPPSData);
void ReceiveData(int fd);
//给mcu发送定位信息
void SendData2MCU(PPSData_t ppsData, PPSData_t correctData);
int ReadMCData();
int SelfTest();
int CorrectAngleAgain();
class own_serial
{
private:
    /* data */
public:
    own_serial();
    ~own_serial();

    int IsOpen(void);
    int Init(int &fd, char *port, int baudrate);
    //int signal_handler_IO(int stuas);
private:
    struct termios termAttr;
    struct sigaction saio;
    int ifSerial = 0;

public:
};
// void SerialSendData(int fd, PPSData_t ppsData, PPSData_t correctData);
const int GET_DATA_LEN = 24;


#endif
