// ----------------------------------------------------------------------
// Include files
// ----------------------------------------------------------------------
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
/* Private typedef -----------------------------------------------------------*/
//typedef  void (*pFunction)(void);

// ----------------------------------------------------------------------
// Struct/Union Types and define
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Static Global Data section variables
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// External Variable 
// ----------------------------------------------------------------------
extern unsigned char bFlagReadKey; //define from stm32f10x_it.c

// ----------------------------------------------------------------------
// Static Prototype Functions
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Static functions
// ----------------------------------------------------------------------
//extern void TX_Test(void);

// ----------------------------------------------------------------------
// Exported functions
// ----------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------


int main(void)
{
float i;
int16_t temp = 0;	
	
	while (TRUE){
		//ReSetVectorTable();
		uComOnChipInitial();  UART1_PRINTF("app1 start\n");
		can_test();  
		Delay_ms(200);	// delay 200ms

		PI5008K_Init();		
	  ADC1Init(); 
	//	GPIO_Configuration();
		do
		{				
			
			PI5008K_UartCmdProcessing();
			//PI5008K_remocon_test(); 
			PI5008K_remocon_test_longkey();
			PI5008K_5Dir_Key_Read();
      PI5008K_5Dir_Center_Key_Led();
			temp = GetADCValue(ADC_Channel_10, 20);
		i = temp * (3.3 / 4096);//2^12
		if(i > 1){
		    GPIO_WriteBit(GPIOB, GPIO_Pin_1, 1);			
			  printf("over voltage > 1v \r\n",i);
		}else{
			  GPIO_WriteBit(GPIOB, GPIO_Pin_1, 0);
		    printf("voltage %.2f\r\n",i);
		}
		}while(1);

    }
		
}

/*  FILE_END_HERE */
