#ifdef __linux
#include "own_serial.h"
int fd = 0;
uint8_t sbuff[64];
uint8_t rbuff[20];

own_serial::own_serial(const char *port,
          uint32_t baudrat)
{
    this_port = port;
    this_baudrate = baudrat;
}
own_serial::~own_serial()
{
}
 
int own_serial::init(int &fd1)
{

    fd1 = open(this_port, O_RDWR| O_NOCTTY| O_NDELAY);
    
    while(fd1 == -1)
    {
		fd1 = open(this_port, O_RDWR| O_NOCTTY| O_NDELAY);
    	perror("open_port: Unable to open serial\n");
    }
    saio.sa_handler = signal_handler_IO;
    //signal_handler_IO(ifSerial);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL; 
    sigaction(SIGIO,&saio,NULL);
    fcntl(fd1, F_SETFL, FNDELAY);
    fcntl(fd1, F_SETOWN, getpid());
    fcntl(fd1, F_SETFL, O_NDELAY| O_ASYNC ); 
    tcgetattr(fd1,&termAttr);
    //baudRate = B115200;          /* Not needed */
    cfsetispeed(&termAttr,this_baudrate);
    cfsetospeed(&termAttr,this_baudrate);
	std::cout << "this_baudrate = " << this_baudrate << std::endl;
    termAttr.c_cflag &= ~PARENB;
    termAttr.c_cflag &= ~CSTOPB;
    termAttr.c_cflag &= ~CSIZE;
    termAttr.c_cflag |= CS8;
    termAttr.c_cflag |= (CLOCAL | CREAD);
    termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
    termAttr.c_oflag &= ~OPOST;
    termAttr.c_cc[VMIN] = 0;
    termAttr.c_cc[VTIME] = 1;
    tcflush(fd1,TCIOFLUSH);
    tcsetattr(fd1,TCSANOW,&termAttr);
    fd = fd1;
    std::cout<<"serial configured....\n";
}

int own_serial::isopen(void)
{
    return(ifSerial);
}

int own_serial::writeData(const char* Data, int size, ...) {
	int c;
	int d;
	float f;

    uint8_t dataC;
    dataUnion<int> dataI;
    dataUnion<float> dataF;  

	uint8_t* buff = new uint8_t[size];
	va_list ap;
	va_start(ap, Data);

	while (*Data != 0)				                          
	{
		if (*Data == '%')									  
		{
			switch (*++Data)
			{
			case 'c':
				c = va_arg(ap, int);
				Data++;
                dataC = static_cast<uint8_t>(c);
				CopyData(&dataC, buff, 1);
				buff++;
				break;

			case 'd':										  
				d = va_arg(ap, int);
				Data++;
				dataI.data = d;
				CopyData(&dataI.dataChar[0], buff, 4);
				buff += 4;
				break;

			case 'f':
				f = va_arg(ap, double);
				Data++;          
				dataF.data = static_cast<float>(f);
				CopyData(&dataF.dataChar[0], buff, 4);
				buff += 4;
			default:
				break;
			}
		}
		else
		{
			*buff = *Data;
			Data++;
			buff++;
		}
	}

	int ifWrite = 0;
    buff -= size;
	std::cout << " size = " << size << std::endl;
	for (size_t i = 0; i < size; i++)
	{
		ifWrite = write(fd, &buff[i], 1);
        std::cout << buff[i] << std::endl;
	}
	delete[] buff;
	return ifWrite;
}

static int cnt =0;
void signal_handler_IO(int status)
{
    uint8_t data;
    cnt++;
    //static int fd =status;
    int nread = read(fd, &rbuff, 4);
    std::cout <<" nread "<<nread<<std::endl;
    if (nread>0)
    {
        std::cout<<rbuff<<" "<<cnt<<std::endl;
    }
    
  
}
void CopyData(uint8_t* origen, uint8_t* afterTreat, int size)
{
    
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}
#endif