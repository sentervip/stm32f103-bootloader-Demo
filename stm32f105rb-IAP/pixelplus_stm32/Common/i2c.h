//----------------------------------------------------------------------------------------------------------------------
// (C) Copyright 2008  Macro Image Technology Co., LTd. , All rights reserved
// 
// This source code is the property of Macro Image Technology and is provided
// pursuant to a Software License Agreement. This code's reuse and distribution
// without Macro Image Technology's permission is strictly limited by the confidential
// information provisions of the Software License Agreement.
//-----------------------------------------------------------------------------------------------------------------------
//
// File Name   		:  I2C.H
// Description 		:  This file contains typedefine for the driver files	
// Ref. Docment		: 
// Revision History 	:

#ifndef		__UCOMI2C_H__
#define		__UCOMI2C_H__
#include	"..\common\board.h"
#include <stdio.h>


BYTE I2C_Write(BYTE ID, BOOL Speed, BYTE dAddr, BYTE rAddr, PBYTE pBuff, WORD bytes);
BYTE I2C_Read(BYTE ID, BOOL Speed, BYTE dAddr, BYTE rAddr, PBYTE pBuff, WORD bytes);
BYTE I2C_Write16(BYTE ID, BOOL Speed, BYTE dAddr, WORD rAddr, PBYTE pBuff, WORD bytes);
BYTE I2C_Read16(BYTE ID, BOOL Speed, BYTE dAddr, WORD rAddr, PBYTE pBuff, WORD bytes);
#endif	/* __UCOMI2C_H__ */
