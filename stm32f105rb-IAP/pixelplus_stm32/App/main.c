#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>
#include "..\Common\common.h"
//#include "..\pi5008k\pi5008k_test.h"
#include "..\pi5008k\pi5008k_func.h"
#include "..\pi5008k\pi5008k_remocon.h"
#include "..\pi5008k\AppIO.h"
#include "..\pi5008k\AppCan.h"
#include "..\pi5008k\AppAdc.h"
#include "..\pi5008k\AppAdc.h"
#include "i2c.h"
uint16_t g_error_flg = OK_CODE;

int main(void)
{
	
	while (TRUE){	
		//ReSetVectorTable();
		Delay_ms(20);	
		uComOnChipInitial();   
		led_line_init();
		can_test(1); 
		ADC1Init(); 
		PI5008K_Init();		  
		CheckPowerStatus();
		GetVersion();
		//I2C_Write(I2C_1, 0, 0x01, 0x20, data, 1); / / for test		
		do
		{			
			
			PI5008K_UartCmdProcessing();
			//PI5008K_remocon_test(); 
			//PI5008K_5Dir_Key_Read();
     //PI5008K_5Dir_Center_Key_Led();			
			PI5008K_remocon_test_longkey();
			CheckPowerStatus();
			poll_line_in();	
			poll_can();
		}while(1);

 }
		
}