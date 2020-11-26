#include <stdio.h>
#include <stdarg.h> 
#include <ctype.h>
#include <string.h>

#include "sys.h"
#include "usart.h"	

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//for usart2
static void long_itoa (long val, int radix, int len, void (*putc) (char))
{
	char c, sgn = 0, pad = ' ';
	char s[20];
	int  i = 0;


	if (radix < 0) {
		radix = -radix;
		if (val < 0) {
			val = -val;
			sgn = '-';
		}
	}
	if (len < 0) {
		len = -len;
		pad = '0';
	}
	if (len > 20) return;
	do {
		c = (char)((unsigned long)val % radix); //cast!
		//if (c >= 10) c += ('A'-10); //ABCDEF
		if (c >= 10) c += ('a'-10); //abcdef
		else c += '0';            //0123456789
		s[i++] = c;
		val = (unsigned long)val /radix; //cast!
	} while (val);
	if (sgn) s[i++] = sgn;
	while (i < len)
		s[i++] = pad;
	do
		(*putc)(s[--i]);
	while (i);
}
static int vfprintf_(void (*putc)(char), const char* str,  va_list arp)
{
	int d, r, w, s, l;  //d=char, r = radix, w = width, s=zeros, l=long
	char *c;            // for the while loop only
	//const char* p;
#ifdef INCLUDE_FLOAT
	float f;
	long int m, w2;
#endif



	while ((d = *str++) != 0) {
		if (d != '%') {//if it is not format qualifier
			(*putc)(d);
			continue;//get out of while loop
		}
		d = *str++;//if it is '%'get next char
		w = r = s = l = 0;
		if (d == '%') {//if it is % print silmpy %
			(*putc)(d);
			d = *str++;
		}
		if (d == '0') {
			d = *str++; s = 1;  //padd with zeros
		}
		while ((d >= '0')&&(d <= '9')) {
			w += w * 10 + (d - '0');
			d = *str++;
		}
		if (s) w = -w;      //padd with zeros if negative

#ifdef INCLUDE_FLOAT
		w2 = 0;
		if (d == '.')
			d = *str++;
		while ((d >= '0')&&(d <= '9')) {
			w2 += w2 * 10 + (d - '0');
			d = *str++;
		}
#endif

		if (d == 's') {// if string
			c = va_arg(arp, char*); //get buffer addres
			//p = c;//debug
			while (*c)
				(*putc)(*(c++));//write the buffer out
			continue;
		}


		//debug

		//while(*p) PutUartCon(*p++);

		if (d == 'c') {
			(*putc)((char)va_arg(arp, int));
			continue;
		}
		if (d == 'u') {     // %ul
			r = 10;
			d = *str++;
		}
		if (d == 'l') {     // long =32bit
			l = 1;
			if (r==0) r = -10;
			d = *str++;
		}
		//		if (!d) break;
		if (d == 'u') r = 10;//     %lu,    %llu
		else if (d == 'd' || d == 'i') {if (r==0) r = -10;}  //can be 16 or 32bit int
		else if (d == 'X' || d == 'x') r = 16;               // 'x' added by mthomas
		else if (d == 'b') r = 2;
		else str--;                                         // normal character

#ifdef INCLUDE_FLOAT
		if (d == 'f' || d == 'F') {
			f=va_arg(arp, double);
			if (f>0) {
				r=10;
				m=(int)f;
			}
			else {
				r=-10;
				f=-f;
				m=(int)(f);
			}
			long_itoa(m, r, w, (putc));
			f=f-m; m=f*(10^w2); w2=-w2;
			long_itoa(m, r, w2, (putc));
			l=3; //do not continue with long
		}
#endif

		if (!r) continue;  //
		if (l==0) {
			if (r > 0){      //unsigned
				unsigned long temp = (unsigned long)va_arg(arp, unsigned);
				//long_itoa((unsigned long)va_arg(arp, int), r, w, (putc)); //needed for 16bit int, no harm to 32bit int
				long_itoa(temp, r, w, (putc)); //needed for 16bit int, no harm to 32bit int
			}
			else            //signed
			{
				//long_itoa((long)va_arg(arp, int), r, w, (putc));
				long temp = (long)va_arg(arp, int);
				long_itoa(temp, r, w, (putc));
			}
		} else if (l==1){  // long =32bit
			//long_itoa((long)va_arg(arp, long), r, w, (putc));        //no matter if signed or unsigned
			long temp = (long)va_arg(arp, long);
			long_itoa(temp, r, w, (putc));        //no matter if signed or unsigned
		}
	}

	return 0;
}
void PutUart2Con(char ch) 
{
    while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
    USART2->DR = (u8) ch;   
}
void UART2_send(char* buf, char len)
{
   for(char i=0;i<len;i++){
	     PutUart2Con(*(buf+i));
	 }
}
void UART2_PRINTF(const char *format, ...)
{
	va_list arg;

	va_start(arg, format);
	vfprintf_((&PutUart2Con), format, arg);
	va_end(arg);
}



//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 usart2_rx_buf[USART_REC_LEN]  __attribute__ ((at(ADDR_RAM_USART_RX)));//���ջ���,���USART_REC_LEN���ֽ�,��ʼ��ַΪ0X20001000.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       	//����״̬���	 
u32 USART_RX_CNT=0;			//���յ��ֽ��� 

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
UART_HandleTypeDef UART1_Handler,UART2_Handler; //UART���
  
//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{	
	//UART1 ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX;		    //��ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
	
	//UART2 ��ʼ������
	UART2_Handler.Instance=USART2;					    //USART2
	UART2_Handler.Init.BaudRate=bound;				    //������
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //����żУ��λ
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()��ʹ��UART1
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  
}

//UART�ײ��ʼ����ʱ��ʹ�ܣ��������ã��ж�����
//�˺����ᱻHAL_UART_Init()����
//huart:���ھ��
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//ģʽҪ����Ϊ��������ģʽ��	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}else if(huart->Instance==USART2)//����Ǵ���2�����д���2 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART2_CLK_ENABLE();			//ʹ��USART2ʱ��
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA2

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//ģʽҪ����Ϊ��������ģʽ��	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA3
		
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART2�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);			//��ռ���ȼ�3�������ȼ�3
	}else;
	
}

/*�����������ֱ�Ӱ��жϿ����߼�д���жϷ������ڲ���*/

//����2�жϷ������
void USART2_IRQHandler(void)                	
{ 
	u8 Res;

	if((__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET))  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res=USART2->DR; 
		if(USART_RX_CNT<USART_REC_LEN)
		{
			usart2_rx_buf[USART_RX_CNT]=Res;
			USART_RX_CNT++;			 									     
		}
	}
	HAL_UART_IRQHandler(&UART2_Handler);	
} 




	


