/*----------------------------------------------------------------------------
 * Name:    CanDemo.c
 * Purpose: CAN example for MCBSTM32E
 * Note(s): possible defines set in "options for target - C/C++ - Define"
 *            __USE_LCD   - enable Output on LCD
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
//#include "CAN.h"                                  /* STM32 CAN adaption layer */
#include "AppIO.h"

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
        
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
//void SysTick_Handler(void) {

  //msTicks++;                        /* increment counter necessary in Delay() */
	
//}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay_Ms (uint32_t dlyTicks) {
  uint32_t curTicks;
	curTicks = 0;
	msTicks = 0;
//  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}



//LED灯和按键引脚初始化
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	
	RCC_APB2PeriphClockCmd( RCC_KEY1|RCC_KEY2|RCC_KEY3|RCC_KEY4 , ENABLE); 	
	RCC_APB2PeriphClockCmd( RCC_LED1|RCC_LED2|RCC_LED3|RCC_LED4 , ENABLE); 	
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE); 	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	
	
	/**
	*	LED 
	*/					 
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED4, &GPIO_InitStructure);
	
	/**
	*	USB ENABLE -> PC5
	*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);
	GPIO_Init(GPIOC, &GPIO_InitStructure);


		/**
	*	DI -> PB12-PPB15
	*/	
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY4, &GPIO_InitStructure);

	LED1_ON();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();

}