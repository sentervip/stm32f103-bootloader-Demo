/*
*Copyright (c) 2019
*All rights reserved.
*
*@file adc.c
*@brief  ��ADC��صĺ���
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
*�������ܣ�ADC��ʼ������
*���������NULL
*���������NULL
*����ֵ��void
*
*/
void  ADC1Init(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);//ʹ��GPIOC��ADC1ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����ADC�ķ�Ƶ���ӣ�6��Ƶ��72/6 = 12
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PC0  adc_ch10  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//����Ϊģ������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//adcģʽ����Ϊ����ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//����adcΪ��ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//��ֹ������⣬ʹ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//���ݶ��뷽ʽ �Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;//�ɼ���ͨ����
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);//����ADC1��У׼�Ĵ���
	while (ADC_GetResetCalibrationStatus(ADC1));//�ȴ���λУ׼����
	ADC_StartCalibration(ADC1);//��ʼУ׼
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�У׼����
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ�����ת������

	
}
/*
*�������ܣ���ȡADCת����ֵ
*���������u8 uCh ADCͨ�� ; u8 uTimes ��ȡ����
*���������NULL
*����ֵ��u16 ������ADC������
*
*/
u16 GetADCValue(u8 uCh, u8 uTimes)
{
	u32 uTempValue = 0;
	u8 uT = 0;
	ADC_RegularChannelConfig(ADC1, uCh, 1, ADC_SampleTime_239Cycles5);//����ʱ��Ϊ239.5��ADC����  ����ͨ��������
	for (uT =0;uT < uTimes; uT++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
		uTempValue += ADC_GetConversionValue(ADC1);
		//DelayMs(5);
	}
	return (uTempValue / uTimes) ;
}	




