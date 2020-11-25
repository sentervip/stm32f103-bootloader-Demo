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
int i;
	
	
	while (TRUE){
		ReSetVectorTable();
		uComOnChipInitial();  UART1_PRINTF("app1 start\n");
		can_test();  
		Delay_ms(200);	// delay 200ms

		PI5008K_Init();		
	
	//	GPIO_Configuration();
		do
		{				
			UART1_PRINTF("loop\n");
			PI5008K_UartCmdProcessing();
			//PI5008K_remocon_test(); 
			PI5008K_remocon_test_longkey();
			PI5008K_5Dir_Key_Read();
      PI5008K_5Dir_Center_Key_Led();
		}while(1);

    }
		
}

/*  FILE_END_HERE */
