#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>

#define CAN0 0
#define CAN1 1

extern int socket_can0;
extern int socket_can1;

void CAN_Init(uint8_t canPort);

#endif
     
