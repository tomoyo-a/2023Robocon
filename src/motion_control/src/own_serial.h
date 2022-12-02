#ifdef __linux
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
#include <string.h>
#include <stdarg.h>
#include <iostream>

#define SERIAL
template<typename T>
union dataUnion 
{
	uint8_t dataChar[4];
	T data;
};
 /* definition of signal handler */
void CopyData(uint8_t* origen, uint8_t* afterTreat, int size);
void signal_handler_IO (int status);  
class own_serial
{
private:
    /* data */
public:
    own_serial(const char * port = "",
          uint32_t baudrate = B921600);
    ~own_serial();
    int isopen(void);
    int init(int &fd1);
	int writeData(const char* Data, int size, ...);
    int this_fd = 0;
private:
    struct termios termAttr;
    struct sigaction saio;
    int ifSerial = 0;
    const char * this_port;
    uint32_t this_baudrate = 9600;  
};

#endif
 
