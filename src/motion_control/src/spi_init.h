#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#define SPI_SPEED (84*1024*1024/32)

int initSPI(void);
int SPIDataRW(int channel,uint8_t *tx_data,uint8_t *rx_data,int len);
int SPISetupMode (int channel, int speed, int mode) ;
int SPISetup     (int channel, int speed) ;
 
#ifdef __cplusplus
}
#endif 
