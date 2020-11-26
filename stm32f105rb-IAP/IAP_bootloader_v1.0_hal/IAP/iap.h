#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  

typedef enum FSM_STATUS{
   // FSM_START = 0,
	  FSM_WAITTING=0,
	  FSM_GET_FILE_LEN,
	  FSM_GET_FILE,
	  FSM_PROGRAM,
	  FSM_BOOT
} FSM_EUM;
typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.
typedef struct {
		uint16_t magic;
		uint16_t running;
		uint16_t app1;
		uint16_t app2;
} MountPointStr;
#define TIMEOUT_2S             2000000
#define TIMEOUT_4S             4000000
#define IAP_CMD_SYNC           0xa5
#define IAP_CMD_ACK_OK         0x01
#define IAP_CMD_ACK_FAIL       0x02
#define IAP_CMD_FILE_LEN       0x03
#define FLASH_CFG_MAGIC     0xa5
#define FLASH_INVALID_DATA  0xff
#define FLASH_CFG_MP_ADDR		0x8002800 
#define FLASH_APP1_ADDR		0x8003000  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)
#define FLASH_APP1_MAX_SIZE  0xe800  // 58KB
											//����0X08003000~0X0801 1800�Ŀռ�ΪIAPʹ��
extern u8 AckOk[4], AckFail[4];											
extern MountPointStr MoutPointTag;

void parse_mountPoint(void);
void iap_load_app(u32 appxaddr);			//ִ��flash�����app����
void iap_load_appsram(u32 appxaddr);		//ִ��sram�����app����
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//��ָ����ַ��ʼ,д��bin
void iap_readback_appbin(u32 appxaddr,u8 *appbuf,u32 appsize);
#endif







































