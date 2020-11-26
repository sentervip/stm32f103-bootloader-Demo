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
	u16 oldcount=0;	//�ϵĴ��ڽ�������ֵ
	uint32_t applenth=0;	//���յ���app���볤��

  HAL_Init();                    	//��ʼ��HAL��    
  Stm32_Clock_Init(RCC_PLL_MUL6); //����ʱ��,72M
  delay_init(72);                 //��ʼ����ʱ����
	uart_init(115200);	 	        //���ڳ�ʼ��Ϊ115200
	LED_Init();		  		        //��ʼ����LED���ӵ�Ӳ���ӿ�
 	KEY_Init();				        //������ʼ��    
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
									    if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000){ //�ж��Ƿ�Ϊ0X08XXXXXX.
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
	u16 oldcount=0;	//�ϵĴ��ڽ�������ֵ
	uint32_t applenth=0;	//���յ���app���볤��

    HAL_Init();                    	//��ʼ��HAL��    
    Stm32_Clock_Init(RCC_PLL_MUL6); //����ʱ��,72M
    delay_init(72);                 //��ʼ����ʱ����
	uart_init(115200);	 	        //���ڳ�ʼ��Ϊ115200
	LED_Init();		  		        //��ʼ����LED���ӵ�Ӳ���ӿ�
 	KEY_Init();				        //������ʼ��    
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
			if(oldcount==USART_RX_CNT)//��������,û���յ��κ�����,��Ϊ�������ݽ������.
			{
				applenth=USART_RX_CNT;
				oldcount=0; 
				USART_RX_CNT=0;
				printf("�û�����������!\r\n");
				printf("���볤��:%dBytes\r\n",applenth);
			}else oldcount=USART_RX_CNT;			
		}
		t++;
		delay_ms(10);
		if(t==30)
		{
			LED0=!LED0;//LED0��˸��ʾ����������
			t=0;
		}	  	 
		key=KEY_Scan(1);
		if(key==WKUP_PRES) //WK_UP��������
		{
			if(applenth)
			{
				printf("��ʼ���¹̼�...\r\n");	
 				if(((*(vu32*)(ADDR_RAM_USART_RX+4))&0xFF000000)==0x08000000)//�ж��Ƿ�Ϊ0X08XXXXXX.
				{	 
					iap_write_appbin(FLASH_APP1_ADDR,usart2_rx_buf,applenth);//����FLASH����   
					printf("�̼��������!\r\n");	
				//	iap_readback_appbin(FLASH_APP1_ADDR,NULL,applenth);
				}else 
				{	   
					printf("��FLASHӦ�ó���!\r\n");
				}
 			}else 
			{
				printf("û�п��Ը��µĹ̼�!\r\n");
			}									 
		} 
		if(key==KEY1_PRES) //KEY1��������
		{
			printf("��ʼִ��FLASH�û�����!!\r\n");
			if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//�ж��Ƿ�Ϊ0X08XXXXXX.
			{	 
				iap_load_app(FLASH_APP1_ADDR);//ִ��FLASH APP����
			}else 
			{
				printf("��FLASHӦ�ó���,�޷�ִ��!\r\n"); 
                printf("\r\n");				
			}									   
		}
		if(key==KEY0_PRES) //KEY0��������
		{
			printf("��ʼִ��SRAM�û�����!!\r\n");
			if(((*(vu32*)(ADDR_RAM_USART_RX+4))&0xFF000000)==0x20000000)//�ж��Ƿ�Ϊ0X20XXXXXX.
			{	 
				iap_load_app(ADDR_RAM_USART_RX);//SRAM��ַ
			}else 
			{
				printf("��SRAMӦ�ó���,�޷�ִ��!\r\n");	
                printf("\r\n");					
			}									 	 
		}				   
		
	}
    
}


