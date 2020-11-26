#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define ADDR_RAM_USART_RX   0X20007000
#define USART_REC_LEN  			28*1024  	//�����������ֽ���58K

void UART2_PRINTF(const char *format, ...);	  	
extern u8  usart2_rx_buf[USART_REC_LEN];
extern u16 USART_RX_STA;         			//����״̬���	
extern UART_HandleTypeDef UART2_Handler; 	//UART���
extern u32 USART_RX_CNT;					//���յ��ֽ��� 

#define RXBUFFERSIZE   1 					//�����С
extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL��USART����Buffer

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void UART2_send(char* buf, char len);
#endif


