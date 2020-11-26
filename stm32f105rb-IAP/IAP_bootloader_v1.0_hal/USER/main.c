#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "stmflash.h"   
#include "iap.h" 

//                sync  cmd DataLen data xor
u8 AckOk[4] =     {0xa5,IAP_CMD_ACK_OK, 0x00,       0x0};
u8 AckFail[4] = {0xa5,IAP_CMD_ACK_FAIL, 0x00,       0x0};
//u8 AckFailed[4] = {0xa5,0x3, 0x03, lenH,lenL,xor, xor};
uint32_t g_timeOld = 0;
uint32_t g_fsm = FSM_WAITTING; 

void polling_timeout(void)
{
	uint32_t count = HAL_GetTick();
    switch(g_fsm){
			case FSM_GET_FILE:
				   if(  count - g_timeOld  > TIMEOUT_4S){
					     printf("FSM_GET_FILE time out");
						   //code ...
					 }
					 break;
			default:
				   if(  count - g_timeOld  > TIMEOUT_2S){
					     printf("FSM_Other time out");
						   //code ...
					 }
					 break;
		  }
}
int main(void)
{
	u8 test[] = {0xa5,1,2,3,4};
	u8 app_xor = 0; 
	u8 key,t;
	u16 oldcount=0;	//老的串口接收数据值
	uint32_t applenth=0;	//接收到的app代码长度

  HAL_Init();                    	//初始化HAL库    
  Stm32_Clock_Init(RCC_PLL_MUL6); //设置时钟,72M
  delay_init(72);                 //初始化延时函数
	uart_init(115200);	 	        //串口初始化为115200
	LED_Init();		  		        //初始化与LED连接的硬件接口
 	KEY_Init();				        //按键初始化    
	printf("IAP TEST\r\n"); 
	g_timeOld = HAL_GetTick();
	while(1){
	    if(g_fsm == FSM_WAITTING){
			    if(USART_RX_CNT >=1 ){
					    if( 0xa5 == usart2_rx_buf[USART_RX_CNT-1]){
								  UART2_send(AckOk,sizeof(AckOk));
								  g_fsm = FSM_GET_FILE_LEN;							                    	
                  g_timeOld = HAL_GetTick();	
                  USART_RX_CNT =0;			
                  printf("s1 enter FSM_WAITTING \n");									
							}else{
							    UART2_send(AckFail,sizeof(AckFail));
							    USART_RX_CNT = 0;
							}
					}
			}else if(g_fsm == FSM_GET_FILE_LEN){   
            if(USART_RX_CNT == 7  && IAP_CMD_FILE_LEN == usart2_rx_buf[1]){					    
								  applenth = usart2_rx_buf[3] <<8 | usart2_rx_buf[4];
								  app_xor = usart2_rx_buf[5];								  
								  UART2_send(AckOk,sizeof(AckOk));
								  g_fsm = FSM_GET_FILE;
							    g_timeOld = HAL_GetTick();		
							    USART_RX_CNT = 0;
							    printf("s2 enter FSM_GET_FILE_LEN :%d,xor:%d \n", applenth, app_xor);			
							}else if(USART_RX_CNT >7 ){
								  UART2_send(AckFail,sizeof(AckFail));
							    USART_RX_CNT = 0;
							}else;		
      }else if(g_fsm == FSM_GET_FILE){  
				    if( USART_RX_CNT == applenth){
							//check: app_xor == XOR
							  if(((*(vu32*)(ADDR_RAM_USART_RX+4))&0xFF000000)==0x08000000){//is 0x08XXXXXX.
								 
									    iap_write_appbin(FLASH_APP1_ADDR,usart2_rx_buf,applenth);//program
									    //	iap_readback_appbin(FLASH_APP1_ADDR,NULL,applenth); 
									    printf("update ok\r\n");	
									    if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000){ //判断是否为0X08XXXXXX.
				                  iap_load_app(FLASH_APP1_ADDR);//goto app
											}else{
												   UART2_send(AckFail,sizeof(AckFail));
											     printf("error, program failed\r\n"); 
											}
								    
								}else {	 
                      UART2_send(AckFail,sizeof(AckFail));									
									    printf("error, invalid .bin\r\n");
								}
						}
			}else;	

      //polling_timeout();			

	}
}
int main2(void)
{
	u8 test[] = {0xa5,1,2,3,4};
	u8 key,t;
	u16 oldcount=0;	//老的串口接收数据值
	uint32_t applenth=0;	//接收到的app代码长度

    HAL_Init();                    	//初始化HAL库    
    Stm32_Clock_Init(RCC_PLL_MUL6); //设置时钟,72M
    delay_init(72);                 //初始化延时函数
	uart_init(115200);	 	        //串口初始化为115200
	LED_Init();		  		        //初始化与LED连接的硬件接口
 	KEY_Init();				        //按键初始化    
	printf("NANO STM32\r\n"); 
	printf("IAP TEST\r\n"); 
  printf("WK_UP:Copy APP2FLASH\r\n"); 
  printf("KEY0:Run SRAM APP\r\n"); 
  printf("KEY1:Run FLASH APP\r\n"); 	
	UART2_PRINTF("start uart2\n");
	
	u32 tick=  HAL_GetTick();
	printf("tick:%d\n",tick);
	while(1)
	{
		tick=  HAL_GetTick();			
		if(USART_RX_CNT)
		{
			if(oldcount==USART_RX_CNT)//新周期内,没有收到任何数据,认为本次数据接收完成.
			{
				applenth=USART_RX_CNT;
				oldcount=0; 
				USART_RX_CNT=0;
				printf("用户程序接收完成!\r\n");
				printf("代码长度:%dBytes\r\n",applenth);
			}else oldcount=USART_RX_CNT;			
		}
		t++;
		delay_ms(10);
		if(t==30)
		{
			LED0=!LED0;//LED0闪烁表示程序在运行
			t=0;
		}	  	 
		key=KEY_Scan(1);
		if(key==WKUP_PRES) //WK_UP按键按下
		{
			if(applenth)
			{
				printf("开始更新固件...\r\n");	
 				if(((*(vu32*)(ADDR_RAM_USART_RX+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
				{	 
					iap_write_appbin(FLASH_APP1_ADDR,usart2_rx_buf,applenth);//更新FLASH代码   
					printf("固件更新完成!\r\n");	
				//	iap_readback_appbin(FLASH_APP1_ADDR,NULL,applenth);
				}else 
				{	   
					printf("非FLASH应用程序!\r\n");
				}
 			}else 
			{
				printf("没有可以更新的固件!\r\n");
			}									 
		} 
		if(key==KEY1_PRES) //KEY1按键按下
		{
			printf("开始执行FLASH用户代码!!\r\n");
			if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
			{	 
				iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
			}else 
			{
				printf("非FLASH应用程序,无法执行!\r\n"); 
                printf("\r\n");				
			}									   
		}
		if(key==KEY0_PRES) //KEY0按键按下
		{
			printf("开始执行SRAM用户代码!!\r\n");
			if(((*(vu32*)(ADDR_RAM_USART_RX+4))&0xFF000000)==0x20000000)//判断是否为0X20XXXXXX.
			{	 
				iap_load_app(ADDR_RAM_USART_RX);//SRAM地址
			}else 
			{
				printf("非SRAM应用程序,无法执行!\r\n");	
                printf("\r\n");					
			}									 	 
		}				   
		
	}
    
}


