/*
*Copyright (c) 2019
*All rights reserved.
*
*@file adc.c
*@brief  与ADC相关的函数
*
*@version 1.0
*@auther willchen
*@date 2019.3.13
*
*/
#include "stm32f10x.h"
#include "STM32F10x_adc.h"
#include "AppAdc.h"
/*
*函数介绍：ADC初始化函数
*输入参数：NULL
*输出参数：NULL
*返回值：void
*
*/
void  ADC1Init(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);//使能GPIOC和ADC1时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC的分频因子，6分频，72/6 = 12
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PC0  adc_ch10  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//配置为模拟输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//adc模式设置为独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//设置adc为非扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//禁止触发检测，使用软件检测
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//数据对齐方式 右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;//采集的通道数
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//重置ADC1的校准寄存器
	while (ADC_GetResetCalibrationStatus(ADC1));//等待复位校准结束
	ADC_StartCalibration(ADC1);//开始校准
	while(ADC_GetCalibrationStatus(ADC1));//等待校准结束
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能软件转换功能

	
}
/*
*函数介绍：获取ADC转换的值
*输入参数：u8 uCh ADC通道 ; u8 uTimes 读取次数
*输出参数：NULL
*返回值：u16 采样的ADC的数据
*
*/
u16 GetADCValue(u8 uCh, u8 uTimes)
{
	u32 uTempValue = 0;
	u8 uT = 0;
	ADC_RegularChannelConfig(ADC1, uCh, 1, ADC_SampleTime_239Cycles5);//采样时间为239.5个ADC周期  规则通道的配置
	for (uT =0;uT < uTimes; uT++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
		uTempValue += ADC_GetConversionValue(ADC1);
		//DelayMs(5);
	}
	return (uTempValue / uTimes) ;
}	




