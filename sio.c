/*****************************Copyright (c)**********************************
**---------------------------------------�ļ���Ϣ----------------------------------------------
**	��  ��  ��: sio.c
** 	��      ��: 1.0
** 	��      ��: 2019/05/
** 	��  ��  ��: 
**	��      ��: ����ͨ��,ѭ����ȡ���ݣ���usleepʵ��ÿ��һ��ʱ����ȡ����
** 	�޸�  ��־:
**********************************************************************************************/
#include	"common.h"
#include "public.h"
#include	"sio.h"
/**********************************************************************************************
**����  ԭ��  :  void SerialRcv(int recv_time)
**����  ˵��  :  recv_time:�����ȡ����ʱ��
				 fd:����������
				 msgid����Ϣ����id
**��  ��  ֵ  :  ��
**˵      ��  :	 ÿ���̶�ʱ����ȡ���ݣ��յ������ݷ��͵�lcd���̵���Ϣ����
**********************************************************************************************/
extern void *SerialRcv(void *data)
//int main(int argc, char **argv)
{
	int len,i;
	uchar buf[512];
	strQmsg send_msg;
	struct SERIALDATA Sdata;
	int  ret;
	struct timeval tv;
	fd_set readset;
	
	printf("SerialRcv ID: %d start his parent is %d\n", getpid(),getppid());	
	Sdata = *((struct SERIALDATA*)data);
	free(data);
	while (1)
	{   
			if(Sdata.recv_time !=0)
				usleep(Sdata.recv_time*1000);
			len = read(Sdata.serial_fd, buf, 256);
		
		if ((len)>0)
		{ 
			if(Sdata.serial_num == SERIALPORTA)
				send_msg.type = COMA_MSG;
			if(Sdata.serial_num == SERIALPORTB)
				send_msg.type = COMB_MSG;
			if(Sdata.serial_num == DBGPORT)
				send_msg.type = DEBUG_MSG;
			memcpy(send_msg.data, buf, len);
			
			//#ifdef __debugcom__
				printf("\n 232 com:%d read %d data\n",Sdata.serial_num,len); 
				for(i=1; i< len+1; i++)
				{
					printf( "%x ", buf[i-1]);  
					if(i%20 == 0)
						printf("\n");
				}
				
					printf("\n");
			//#endif   
			Msgsnd(Sdata.msg_id, &send_msg, len,0);
		}
		else if(len <0)
		 printf("recv error \n");
	}
}

/**********************************************************************************************
**����  ԭ��  :  int OpenPort(int port_num)
**����  ˵��  :  port_num :���ں�
**��  ��  ֵ  :  �ɹ�:����������;ʧ��:FALSE
**˵      ��  :	 ��ָ���Ĵ���,����RAWģʽ
**********************************************************************************************/
int OpenPort( char port_num)
{
	int fd; 
	char *ptr_portname;
	switch(port_num)
	{
		case 0:
			ptr_portname = "/dev/tty0";
			break;
		case 1:
			ptr_portname = "/dev/ttymxc1"; 
			break;
		case 2:
			ptr_portname = "/dev/ttyO3";
			break;
		case 3:
			ptr_portname = "/dev/ttyO2"; 
			break;  
		case 4:
			ptr_portname = "/dev/ttyO1";
			break;
		default:
			err_msg("Unsupported port num %d",port_num); 
			return (FALSE);  
	}
	//fd = open(ptr_portname, O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(ptr_portname, O_RDWR|O_NOCTTY|O_NDELAY,S_IRWXU|S_IRGRP|S_IROTH);
	printf("open serial port %s\n",ptr_portname);
	//fd = open(ptr_portname, O_RDWR);//|O_NOCTTY|O_NDELAY,S_IRWXU|S_IRGRP|S_IROTH
	if (fd == -1)
	{
		err_msg("open_port: Unable to open the serial port %d %s",port_num,ptr_portname);
		return(FALSE);
	}
	return (fd);
	
}
/**********************************************************************************************
**����  ԭ��  :  SetSerial(int fd,int databits,int stopbits,int parity,speed_t speed)
**����  ˵��  :  fd  ����  int  �򿪵Ĵ����ļ����
              databits ����  int ����λ   ȡֵ Ϊ 7 ����8
              stopbits ����  int ֹͣλ   ȡֵΪ 1 ����2
              parity   ����  int  Ч������ ȡֵΪN,E,O,,S
              speed    ����  speed_t ȡֵ B19200 B9600��
**��  ��  ֵ  :  FALSE(-1); TRUE(0)
**˵      ��  :	 ���ô�������λ��ֹͣλ��Ч��λ�Ͳ�����
**********************************************************************************************/
int SetSerial(int fd,char databits,char stopbits,char parity,speed_t speed)
{ 
	struct termios options; 
	
	bzero(&options, sizeof(options));
	tcflush(fd, TCIOFLUSH);  /*ˢ������������������Ѿ��յ�,���û�����δ��*/   
	cfsetispeed(&options, speed);  
	cfsetospeed(&options, speed);   
   
	options.c_cflag &= ~CSIZE; /*������λ����Ȼ������*/
	switch (databits) /*��������λ��*/
	{   
		case 7:		
			options.c_cflag |= CS7; 
			break;
		case 8:     
			options.c_cflag |= CS8;
			break;   
		default:    
			err_msg("Unsupported data size"); 
			return (FALSE);  
	}
	switch (parity) 
	{   
  		case 'n':
  		case 'N':    
  			options.c_cflag &= ~PARENB;   /* Clear parity enable */
  			options.c_iflag &= ~INPCK;    /* Disable parity checking */ 
  			break;  
  		case 'o':   
  		case 'O':     
  			options.c_cflag |= (PARODD | PARENB); /* ����Ϊ��Ч��*/  
  			options.c_iflag |= INPCK;             /* Enable parity checking */ 
  			break;  
  		case 'e':  
  		case 'E':   
  			options.c_cflag |= PARENB;     /* Enable parity */    
  			options.c_cflag &= ~PARODD;    /* ת��ΪżЧ��*/     
  			options.c_iflag |= INPCK;      /* Enable parity checking */
  			break;
  		case 'S': 
  		case 's':  /*as no parity*/   
  	  		options.c_cflag &= ~PARENB;
  			options.c_cflag &= ~CSTOPB;
  			break;  
  		default:   
  			err_msg("Unsupported parity");    
			return (FALSE);  
	}  
/* ����ֹͣλ*/  
	switch (stopbits)
	{   
		case 1:    
  			options.c_cflag &= ~CSTOPB;  
  			break;  
		case 2:    
			options.c_cflag |= CSTOPB;  
			break;
		default:    
			err_msg("Unsupported stop bits");  
  			return (FALSE); 
	} 
/* Set input parity option */ 
	//if ((parity != 'n') && (parity != 'N'))   
	//	options.c_iflag |= ( INPCK | ISTRIP | PARMRK); /*������ż��飬�����Ѿ�����*/
	//	options.c_iflag |= ( INPCK | PARMRK); /*������ż��飬ISTRIP����ʱ�����,PARMRK���ú���0xff��෢��һ��0xff*/
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);/*�����ڹ��ܶ�����ҪRAWģʽ*/
	options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	options.c_oflag &= ~OPOST;   /*Output raw output*/
	options.c_cc[VTIME] = 0; /* ���ó�ʱ10 seconds*/   
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
	options.c_cflag |= (CLOCAL |CREAD);
	if (tcsetattr(fd,TCSANOW,&options) != 0)   
	{ 
		err_msg("Setup Serial attribute error");           
		perror("err:");
		return (FALSE);  
	}
	else
	{ 
		tcflush(fd,TCIOFLUSH); 
 		return (TRUE);
 	}  
}

/**********************************************************************************************
**����  ԭ��  :  SetRS485(int fd,int databits,int stopbits,int parity,speed_t speed)
**����  ˵��  :  fd  ����  int  �򿪵Ĵ����ļ����
    
              parity   ����  int  Ч������ ȡֵΪ0:��У��,1:odd 2:even
              speed    ����  int  ȡֵ 19200 9600��
**��  ��  ֵ  :  FALSE(-1); TRUE(0)
**˵      ��  :	 ���ô�������λ��ֹͣλ��Ч��λ�Ͳ�����
**********************************************************************************************/
int SetRS485(int fd,char parity,int speed)
{ 
	RS485_SET_ARG arg;
	
  arg.baud_rate = speed;
  arg.parity=parity;
  if(ioctl(fd, 0x6503, &arg) < 0)
  {
    err_msg("ioctl(0x6503) failed!");
    return -1;
  }
  return 0;
}

//���ڳ�ʼ��
int UART_Init(UART_HANDLE *pHandle, char port_num, char databits,char stopbits,char parity,speed_t speed)
{
	
	char *ptr_portname,*io_dir_name;
	int ret;
	
	pHandle->fd = -1;

	if(pHandle==NULL )
	{
		printf("%s(%d)Check the input parameters!\r\n",__FILE__ , __LINE__);
		return -1;
	}	
	
	switch(port_num)
	{
		case 0:
			ptr_portname = "/dev/tty0";
			break;
		case 1:
			ptr_portname = "/dev/ttymxc1"; 
			break;
		case 2:
			ptr_portname = "/dev/ttyO3";  
			break;
		case 3:
			ptr_portname = "/dev/ttyO2"; 
			break;  
		case 4:
			ptr_portname = "/dev/ttyO1";
			break;
		default:
			printf("Unsupported port num %d",port_num); 
			return (FALSE);  
	}
	pHandle->fd  = open(ptr_portname, O_RDWR|O_NOCTTY|O_NDELAY,S_IRWXU|S_IRGRP|S_IROTH);
	printf("open serial port %s ok \n",ptr_portname);
	strcpy(pHandle->UartDeviceName, ptr_portname);	
	pHandle->port_num = port_num;          
	if (pHandle->fd < 0)
	{
		printf("open_port: Unable to open the serial port %d %s",port_num,ptr_portname);
		return(FALSE);
	}
	
	else
	{
		ret = SetSerial(pHandle->fd, databits, stopbits, parity,speed); 
		if(ret == FALSE)
		{
			printf("set serial port %s failed \n",ptr_portname);
			return(FALSE);
		}
		else
			printf("set serial port %s success \n",ptr_portname);
	}

	return 0;
}


//���ڷ�������
int UART_SendData(UART_HANDLE *pHandle, u8 *pDataBuff, u32 len,uchar delay_ms)
{
	//printf("Uart(%s) Send Data 1111\n",pHandle->UartDeviceName);
	if(pHandle == NULL || pDataBuff == NULL)
	{
		printf("%s(%d)Check the input parameters!\r\n",__FILE__ , __LINE__);
		return -1;
	} 
	//printf("Uart(%s) Send Data \n",pHandle->UartDeviceName);
	if(write(pHandle->fd, pDataBuff, len) < 0)
	{
		printf("Uart(%s) Send Data Error : %s(%d)\n",pHandle->UartDeviceName,  strerror(errno), errno);
		return errno;
	}
	usleep(delay_ms*1000);
	return 0;
}

//���ڷ����ַ���
int UART_SendString(UART_HANDLE *pHandle, const char *pString)
{
	u32 len;

	if(pHandle == NULL || pString == NULL)
	{
		printf("%s(%d)Check the input parameters!\r\n",__FILE__ , __LINE__);
		return -1;
	}
	len = strlen(pString);
	if(write(pHandle->fd, (u8*)pString, len) <= 0)
	{
		printf("Uart(%s) Send String Error : %s(%d)\n",pHandle->UartDeviceName, strerror(errno), errno);
		return errno;
	}

	return 0;
}


//���ڶ�ȡ����
int UART_ReadData(UART_HANDLE *pHandle, u8 *pDataBuff,u32 DataBuffSize)
{
	int len;

	if(pHandle == NULL || pDataBuff == NULL)
	{
		printf("%s(%d)Check the input parameters!\r\n",__FILE__ , __LINE__);
		return 0;
	}

	len = read(pHandle->fd, pDataBuff, DataBuffSize);
	if(len < 0) //��������ȡ�᷵��Resource temporarily unavailable������11
	{
		//printf("Uart(%s) Read Data Error : %s(%d)\n",pHandle->UartDeviceName,  strerror(errno), errno);
		return 0;
	}

	return len;
}



//������ڽ��ջ�����
void UART_ClearRxData(UART_HANDLE *pHandle)
{
	int len;
	static u8 buff[512+4];
	int i = 0;

	if(pHandle == NULL)
	{
		printf("%s(%d)Check the input parameters!\r\n",__FILE__ , __LINE__);
		return;
	}

	for(i = 0;i < 100000;i ++)
	{
		len = read(pHandle->fd, buff, 512);
		if(len <= 0)//��������ȡ�᷵��Resource temporarily unavailable������11
		{
			//printf("Uart(%s) Clear Data Error : %s(%d)\n",pHandle->UartDeviceName,  strerror(errno), errno);
			break;
		}
	}
}
