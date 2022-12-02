#ifndef _MCUCOMM_H
#define _MCUCOMM_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#define MCU_SPI_LENGTH (62)
#define MCU_SPI_UINT8_CNT (9)
#define MCU_SPI_FLOAT_CNT (12)
extern pthread_t spi_thread_id;

void WaitMcuPrepare(void);
void Communicate2Mcu(void);
void WriteSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* txData, ...);
void ReadSpiData(uint8_t uint8DataCnt, uint8_t floatDataCnt, uint8_t dataLength,char* rxData, ...);
int SPIInit(void);
void NewSpiThread(void);
#endif // _MCUCOMM_H

