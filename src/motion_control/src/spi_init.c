#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi_init.h"
#include "gpio.h"
 
 
static const char   *spiDev0  = "/dev/spidev0.0" ;
static const char   *spiDev1  = NULL;
static  uint8_t     spiBPW   = 8;
static  uint16_t    spiDelay = 2;
 
static uint32_t     spiSpeeds [2] ;
static int          spiFds [2] ;

 
int initSPI(void)
{
  int spiFd;
  spiFd=SPISetup(0,SPI_SPEED); //初始化SPI通道0，并设置为最大速度500000hz
  while(spiFd==-1)
  {
    spiFd=SPISetup(0,SPI_SPEED); //初始化SPI通道0，并设置为最大速度500000hz
    printf("init spi failed!\n");
  }
  return spiFd;
}



/*
* SPIDataRW:
*    Write and Read a block of data over the SPI bus.
*    Note the data ia being read into the transmit buffer, so will
*    overwrite it!
*    This is also a full-duplex operation.
*********************************************************************************
*********************************************************************************/
int SPIDataRW(int channel, uint8_t *tx_data, uint8_t *rx_data,int len)
{
  int i = 0;
 
  struct spi_ioc_transfer spi ;
 
  channel &= 1 ;
 
  memset(&spi, 0, sizeof (spi));
 
  spi.tx_buf        = (unsigned long)tx_data ;
  spi.rx_buf        = (unsigned long)rx_data ;
  spi.len           = len ;
  spi.delay_usecs   = spiDelay ;
  spi.speed_hz      = spiSpeeds [channel] ;
  spi.bits_per_word = spiBPW ;
  return ioctl (spiFds [channel], SPI_IOC_MESSAGE(1), &spi) ; //SPI_IOC_MESSAGE(1)的1表示spi_ioc_transfer的数量

}
 
/*
* SPISetupMode:
*    Open the SPI device, and set it up, with the mode, etc.
*********************************************************************************
*********************************************************************************/
 
int SPISetupMode (int channel, int speed, int mode)
{
  int fd ;
  
  if ((fd = open (channel == 0 ? spiDev0 : spiDev1, O_RDWR)) < 0)
  {
    printf("Unable to open SPI device: %s\n", strerror (errno)) ;
    return -1;
  }
 
  spiSpeeds [channel] = speed ;
  spiFds    [channel] = fd ;
  
/*
* 设置spi的读写模式：
*  Mode 0： CPOL=0, CPHA=0
*  Mode 1： CPOL=0, CPHA=1
*  Mode 2： CPOL=1, CPHA=0
*  Mode 3： CPOL=1, CPHA=1
*  这里我们默认设置为模式0
*********************************************************************************
*/
  if (ioctl (fd, SPI_IOC_WR_MODE, &mode) < 0)                     
  {                                                               
    printf("Can't set spi mode: %s\n", strerror (errno)) ;         
    return -1;                                                    
  }                                                               
 
  if (ioctl (fd, SPI_IOC_RD_MODE, &mode) < 0)                     
  {                                                               
    printf("Can't get spi mode: %s\n", strerror (errno)) ;        
    return -1;                                                 
  }    
  
/*
* spi的读写bit/word设置可写
*    这里设置为8个位为一个字节
*********************************************************************************
*/
  if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0)          
  {                                                               
    printf("Can't set bits per word: %s\n", strerror (errno))  ;  
    return -1;                                                    
  }                                                              
  
  if (ioctl (fd, SPI_IOC_RD_BITS_PER_WORD, &spiBPW) < 0)          
  {                                                               
    printf("Can't get bits per word: %s\n", strerror (errno))  ;  
    return -1;                                                   
  }   
  
/*
* 设置spi读写速率
*********************************************************************************
*/
  if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
  {
    printf("Can't set max speed hz: %s\n", strerror (errno));
    return -1;
  }
  
  if (ioctl (fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
  {
    printf("Can't get max speed hz: %s\n", strerror (errno));
    return -1;
  }
  
  return fd ;
}
 
 
/*
* SPISetup:
*    Open the SPI device, and set it up, etc. in the default MODE 0
*********************************************************************************
*********************************************************************************/
 
int SPISetup (int channel, int speed)
{
  return SPISetupMode (channel, speed, 3) ;

}
