/**
 * AS - the open source Automotive Software on https://github.com/parai
 *
 * Copyright (C) 2015  AS <parai@foxmail.com>
 *
 * This source code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation; See <http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt>.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
/* ============================ [ INCLUDES  ] ====================================================== */
#ifdef __WINDOWS__
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <windows.h>
#undef SLIST_ENTRY
#endif
#include "rs232.h"
#include "Std_Types.h"
#include "lascanlib.h"
#include <sys/queue.h>
#include <pthread.h>
#include "asdebug.h"
#ifdef __LINUX__
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
/* ============================ [ MACROS    ] ====================================================== */
#define CAN_SERIAL_CACHE_SIZE  64
#define AS_LOG_RS232   AS_LOG_STDOUT

/* QEMU TCP 127.0.0.1:1103 't' = 0x74, 'c' = 0x63, 'p' = 0x70 */
#define CAN_TCP_SERIAL_PORT 0x746370
/* ============================ [ TYPES     ] ====================================================== */
struct Can_SerialHandle_s
{
	uint32_t busid;
	uint32_t port;
	uint32_t baudrate;
	char r_cache[CAN_SERIAL_CACHE_SIZE+1];
	int  r_size;
	int  s; /* socket handle used only by QEMU TCP 127.0.0.1:1103 */
	can_device_rx_notification_t rx_notification;
	STAILQ_ENTRY(Can_SerialHandle_s) entry;
};
struct Can_SerialHandleList_s
{
	pthread_t rx_thread;
	volatile boolean   terminated;
	STAILQ_HEAD(,Can_SerialHandle_s) head;
};
/* ============================ [ DECLARES  ] ====================================================== */
static boolean serial_probe(uint32_t busid,uint32_t port,uint32_t baudrate,can_device_rx_notification_t rx_notification);
static boolean serial_write(uint32_t port,uint32_t canid,uint32_t dlc,uint8_t* data);
static void serial_close(uint32_t port);
static void * rx_daemon(void *);
/* ============================ [ DATAS     ] ====================================================== */
const Can_DeviceOpsType can_serial_ops =
{
	.name = "serial",
	.probe = serial_probe,
	.close = serial_close,
	.write = serial_write,
};
static struct Can_SerialHandleList_s* serialH = NULL;
/* ============================ [ LOCALS    ] ====================================================== */
static struct Can_SerialHandle_s* getHandle(uint32_t port)
{
	struct Can_SerialHandle_s *handle,*h;
	handle = NULL;
	if(NULL != serialH)
	{
		STAILQ_FOREACH(h,&serialH->head,entry)
		{
			if(h->port == port)
			{
				handle = h;
				break;
			}
		}
	}
	return handle;
}
static boolean serial_probe(uint32_t busid,uint32_t port,uint32_t baudrate,can_device_rx_notification_t rx_notification)
{
	boolean rv = TRUE;;
	struct Can_SerialHandle_s* handle;
	if(NULL == serialH)
	{
		serialH = malloc(sizeof(struct Can_SerialHandleList_s));
		asAssert(serialH);
		STAILQ_INIT(&serialH->head);

		serialH->terminated = TRUE;
	}

	handle = getHandle(port);

	if(handle)
	{
		ASWARNING("CAN SERIAL port=%d is already on-line, no need to probe it again!\n",port);
		rv = FALSE;
	}
	else
	{
		if(CAN_TCP_SERIAL_PORT == port)
		{
			int s;
			struct sockaddr_in addr;
			#ifdef __WINDOES__
			{
				WSADATA wsaData;
				WSAStartup(MAKEWORD(2, 2), &wsaData);
			}
			#endif

			memset(&addr,0,sizeof(addr));
			addr.sin_family = AF_INET;
			addr.sin_addr.s_addr = inet_addr("127.0.0.1");
			addr.sin_port = htons(1103);

			if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
				ASWARNING("CAN Serial TCP open failed!\n");
				rv = FALSE;
			}

			if( rv )
			{
				/* Connect to server. */
				if(connect(s, (struct sockaddr*) & addr, sizeof (addr)) < 0)
				{
					#ifdef __WINDOWS__
					ASWARNING("CAN Serial TCP connect failed: %d\n", WSAGetLastError());
					closesocket(s);
					#else
					ASWARNING("CAN Serial TCP connect failed!\n");
					close(s);
					#endif
					rv = FALSE;
				}
			}

			if(rv)
			{
				handle = malloc(sizeof(struct Can_SerialHandle_s));
				asAssert(handle);
				handle->s = s;
				handle->busid = busid;
				handle->port = port;
				handle->baudrate = baudrate;
				handle->rx_notification = rx_notification;
				handle->r_size = 0;
				handle->r_cache[CAN_SERIAL_CACHE_SIZE] = '\0';
				STAILQ_INSERT_TAIL(&serialH->head,handle,entry);
				ASLOG(STDOUT,"CAN Serial TCP open OK\n");
			}
		}
		else if( 0 == RS232_OpenComport(port,baudrate,"8N1"))
		{	/* open port OK */
			handle = malloc(sizeof(struct Can_SerialHandle_s));
			asAssert(handle);
			handle->busid = busid;
			handle->port = port;
			handle->baudrate = baudrate;
			handle->rx_notification = rx_notification;
			handle->r_size = 0;
			handle->r_cache[CAN_SERIAL_CACHE_SIZE] = '\0';
			STAILQ_INSERT_TAIL(&serialH->head,handle,entry);
			ASLOG(STDOUT,"CAN Serial open port %d OK\n",port);
		}
		else
		{
			ASWARNING("CAN SERIAL port=%d is is not able to be opened!\n",port);
			rv = FALSE;
		}
	}

	if( (TRUE == serialH->terminated) &&
		(FALSE == STAILQ_EMPTY(&serialH->head)) )
	{
		if( 0 == pthread_create(&(serialH->rx_thread),NULL,rx_daemon,NULL))
		{
			serialH->terminated = FALSE;
		}
		else
		{
			asAssert(0);
		}
	}

	return rv;
}
static boolean serial_write(uint32_t port,uint32_t canid,uint32_t dlc,uint8_t* data)
{
	boolean rv = TRUE;
	struct Can_SerialHandle_s* handle = getHandle(port);

	if(handle != NULL)
	{
		char string[512];
		uint32_t i;
		int size = snprintf(string,sizeof(string),"SCAN%02X%04X%02X",handle->busid,canid,dlc);
		for(i=0;i<dlc;i++)
		{
			size += snprintf(&string[size],sizeof(string)-size,"%02X",data[i]);
		}

		string[size++] = '\n';
		string[size] = '\0';
		ASLOG(OFF,"can serial write:: %s",string);

		if( CAN_TCP_SERIAL_PORT == handle->port)
		{
			if(size == send(handle->s, (const char*)string, size,0))
			{
				/* send OK */
			}
			else
			{
				ASWARNING("CAN Serial TCP send message failed!\n");
			}
		}
		else if(size == RS232_SendBuf((int)handle->port,(unsigned char*)string,size))
		{
			/* send OK */
		}
		else
		{
			rv = FALSE;
			ASWARNING("CAN Serial port=%d<%d> send message failed!\n",port,handle->port);
		}
	}
	else
	{
		rv = FALSE;
		ASWARNING("CAN Serial port=%d is not on-line, not able to send message!\n",port);
	}

	return rv;
}
static void serial_close(uint32_t port)
{
	struct Can_SerialHandle_s* handle = getHandle(port);
	if(NULL != handle)
	{
		STAILQ_REMOVE(&serialH->head,handle,Can_SerialHandle_s,entry);

		if(CAN_TCP_SERIAL_PORT == port)
		{
			#ifdef __WINDOWS__
			closesocket(handle->s);
			#else
			close(handle->s);
			#endif
		}
		else
		{
			RS232_CloseComport(handle->port);
		}

		free(handle);

		if(TRUE == STAILQ_EMPTY(&serialH->head))
		{
			serialH->terminated = TRUE;
		}
	}
}
static uint32_t IntH(char chr)
{
	uint32_t v;
	if( (chr>='0') && (chr<='9'))
	{
		v= chr - '0';
	}
	else if( (chr>='A') && (chr<='F'))
	{
		v= chr - 'A' + 10;
	}
	else if( (chr>='a') && (chr<='f'))
	{
		v= chr - 'a' + 10;;
	}
	else
	{
		ASWARNING("CAN serial receiving invalid data '0x%02X', cast to 0\n",chr);
		v = 0;
	}

	return v;
}
static void rx_notifiy(struct Can_SerialHandle_s* handle)
{
	uint32_t canid;
	uint32_t busid;
	uint32_t dlc;
	uint8_t  data[8];
	uint32_t i;
	if( (handle->r_size >= 12) &&
		('S' == handle->r_cache[0]) &&
		('C' == handle->r_cache[1]) &&
		('A' == handle->r_cache[2]) &&
		('N' == handle->r_cache[3]) )
	{
		ASLOG(OFF,"can serial rx:: %s",handle->r_cache);
		busid = IntH(handle->r_cache[4])*16 + IntH(handle->r_cache[5]);
		canid = IntH(handle->r_cache[6])*16*16*16 + IntH(handle->r_cache[7])*16*16 + IntH(handle->r_cache[8])*16 + IntH(handle->r_cache[9]);
		dlc   = IntH(handle->r_cache[10])*16 + IntH(handle->r_cache[11]);
		if((dlc>0) &&(dlc<=8))
		{
			for(i=0;i<dlc;i++)
			{
				data[i] = IntH(handle->r_cache[12+2*i])*16 + IntH(handle->r_cache[13+2*i]);
			}
			if(NULL != handle->rx_notification)
			{
				handle->rx_notification(busid,canid,dlc,data);
			}
		}
		else
		{
			ASWARNING("CAN serial port=%d receiving data with invalid dlc = %d\n",handle->port,dlc);
		}
	}
	else
	{
		ASLOG(RS232,"%s\n",handle->r_cache);
	}

	handle->r_size = 0;
}
static void * rx_daemon(void * param)
{
	(void)param;
	struct Can_SerialHandle_s* handle;
	while(FALSE == serialH->terminated)
	{
		STAILQ_FOREACH(handle,&serialH->head,entry)
		{
			char chr;
			int size;
			do
			{
				if(CAN_TCP_SERIAL_PORT == handle->port)
				{
					size = recv(handle->s,&chr,1,0);
				}
				else
				{
					size = RS232_PollComport((int)handle->port,(unsigned char*)&chr,1);
				}

				if(1u == size)
				{
					handle->r_cache[handle->r_size++] = chr;
					if('\n' == chr)
					{
						rx_notifiy(handle);
					}

					if(handle->r_size >= CAN_SERIAL_CACHE_SIZE)
					{
						handle->r_size = 0;
						for(int i=0;i<CAN_SERIAL_CACHE_SIZE;i++)
						{
							if('\0' == handle->r_cache[i])
							{
								handle->r_cache[i] = ' ';
							}
						}
						ASWARNING("CAN serial port=%d receiving invalid data, buffer over-run:: %s\n",handle->port,handle->r_cache);
					}
				}
			}while(1u == size);
		}
	}

	return NULL;
}

/* ============================ [ FUNCTIONS ] ====================================================== */


