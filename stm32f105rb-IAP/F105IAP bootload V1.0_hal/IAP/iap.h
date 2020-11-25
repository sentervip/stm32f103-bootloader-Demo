#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  

typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.


#define FLASH_APP1_ADDR		0x8003000  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
#define FLASH_APP1_MAX_SIZE  0xe800  // 58KB
											//����0X08003000~0X0801 1800�Ŀռ�ΪIAPʹ��
											

void iap_load_app(u32 appxaddr);			//ִ��flash�����app����
void iap_load_appsram(u32 appxaddr);		//ִ��sram�����app����
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//��ָ����ַ��ʼ,д��bin
void iap_readback_appbin(u32 appxaddr,u8 *appbuf,u32 appsize);
#endif







































