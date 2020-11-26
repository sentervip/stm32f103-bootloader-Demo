#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define ADDR_RAM_USART_RX   0X20007000
#define USART_REC_LEN  			28*1024  	//定义最大接收字节数58K

void UART2_PRINTF(const char *format, ...);	  	
extern u8  usart2_rx_buf[USART_REC_LEN];
extern u16 USART_RX_STA;         			//接收状态标记	
extern UART_HandleTypeDef UART2_Handler; 	//UART句柄
extern u32 USART_RX_CNT;					//接收的字节数 

#define RXBUFFERSIZE   1 					//缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void UART2_send(char* buf, char len);
#endif


