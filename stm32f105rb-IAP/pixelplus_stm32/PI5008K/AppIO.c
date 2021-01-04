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
#include "pi5008k_func.h"

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
static  uint8_t g_line_in_old = 0x07;
        
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
void led_off(uint16_t pin) 
{
   GPIO_ResetBits(PORT_LED,pin);
}
void led_on(uint16_t pin) 
{
   GPIO_SetBits(PORT_LED,pin);
}
//#define LED_ON() GPIO_ResetBits(PORT_LED1,LED1_PIN)
//#define LED1_OFF() GPIO_SetBits(PORT_LED1,LED1_PIN)
void led_line_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/**
	*	LED 
	*/					 
	GPIO_InitStructure.GPIO_Pin = G_LED | R_LED;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED, &GPIO_InitStructure);
	led_off(R_LED);
	led_off(G_LED);
	
	/** PWR_EN */
	GPIO_InitStructure.GPIO_Pin = CAM_PWR_EN | LCD_PWR_EN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_EN_OUT, &GPIO_InitStructure);

	
   /** LINE IN */
	GPIO_InitStructure.GPIO_Pin = GEAR_LINE_IN | L_LINE_IN |R_LINE_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_LINE_IN, &GPIO_InitStructure);
}

void poll_line_in(void)
{
   uint16_t data = 0;
	
	data = GPIO_ReadInputData(PORT_LINE_IN);
	data &= (GEAR_LINE_IN | L_LINE_IN |R_LINE_IN);
	data >>=7;
	if(data != g_line_in_old){
	    g_line_in_old = data;
		  switch(data){			    
				  case 2:  data = VIEW_4; break;
				  case 3:  data = VIEW_2; break;
			    case 4:  data = VIEW_6; break;
				  case 5:  data = VIEW_8; break;				
				  case 6:  data = VIEW_5; break;	
				  case 7:  data = VIEW_DEFAULT; break;
				  default: break;
			}
		  PI5008K_Uart_Con_Remote_Cmd_View(data);	 
	}
}