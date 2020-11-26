#include <stdio.h>
#include <stdarg.h> 
#include <ctype.h>
#include <string.h>

#include "sys.h"
#include "usart.h"	

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
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
    while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
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



//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 usart2_rx_buf[USART_REC_LEN]  __attribute__ ((at(ADDR_RAM_USART_RX)));//接收缓冲,最大USART_REC_LEN个字节,起始地址为0X20001000.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       	//接收状态标记	 
u32 USART_RX_CNT=0;			//接收的字节数 

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler,UART2_Handler; //UART句柄
  
//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound)
{	
	//UART1 初始化设置
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //波特率
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART1_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART1_Handler.Init.Mode=UART_MODE_TX;		    //发模式
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()会使能UART1
	
	//UART2 初始化设置
	UART2_Handler.Instance=USART2;					    //USART2
	UART2_Handler.Init.BaudRate=bound;				    //波特率
	UART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	UART2_Handler.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	UART2_Handler.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	UART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	UART2_Handler.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&UART2_Handler);					    //HAL_UART_Init()会使能UART1
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  
}

//UART底层初始化，时钟使能，引脚配置，中断配置
//此函数会被HAL_UART_Init()调用
//huart:串口句柄
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART1)//如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE();			//使能USART1时钟
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//模式要设置为复用输入模式！	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn,3,3);			//抢占优先级3，子优先级3
#endif	
	}else if(huart->Instance==USART2)//如果是串口2，进行串口2 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART2_CLK_ENABLE();			//使能USART2时钟
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_2;			//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA2

		GPIO_Initure.Pin=GPIO_PIN_3;			//PA3
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//模式要设置为复用输入模式！	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化PA3
		
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//使能USART2中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);			//抢占优先级3，子优先级3
	}else;
	
}

/*下面代码我们直接把中断控制逻辑写在中断服务函数内部。*/

//串口2中断服务程序
void USART2_IRQHandler(void)                	
{ 
	u8 Res;

	if((__HAL_UART_GET_FLAG(&UART2_Handler,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
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




	


