//----------------------------------------------------------------------------------------------------------------------
// (C) Copyright 2008  Macro Image Technology Co., LTd. , All rights reserved
// 
// This source code is the property of Macro Image Technology and is provided
// pursuant to a Software License Agreement. This code's reuse and distribution
// without Macro Image Technology's permission is strictly limited by the confidential
// information provisions of the Software License Agreement.
//-----------------------------------------------------------------------------------------------------------------------
//
// File Name   		:  COMMON.H
// Description 		:  This file contains typedefine for the driver files	
// Ref. Docment		: 
// Revision History 	:

#ifndef		__COMMON_H__
#define		__COMMON_H__

// -----------------------------------------------------------------------------
// Include files
// -----------------------------------------------------------------------------

#ifndef		__BOARD_H__
#include	"..\common\board.h"
#endif
#ifndef		__UCOMI2C_H__
#include	"..\common\i2c.h"
#endif

#define		UARTprintf		printf
#include	<stdio.h>

enum ENUM_ERROR{
     OK_CODE = 0,
	   OVERPOWER_CODE =1,
	   LOWPOWER_CODE=2,
};

extern uint16_t g_error_flg;
void uComOnChipInitial(void);
void Delay_ms(WORD nTime);
void Delay_10us(WORD nTime);




#endif	/* __COMMON_H__ */
