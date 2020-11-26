#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "stmflash.h"
#include "iap.h"

iapfun jump2app; 
MountPointStr MoutPointTag = {0};
u16 iapbuf[FLASH_PAGE_SIZE/2] ={0xff};   
u16 iapbuf2[FLASH_PAGE_SIZE/2] ={0xff};  
//                sync  cmd DataLen data xor


void parse_mountPoint(void)
{
   STMFLASH_Read(FLASH_CFG_MP_ADDR, &MoutPointTag.magic, 4);
	 if(MoutPointTag.magic == FLASH_CFG_MAGIC){
	     if(MoutPointTag.running == 1){
				    MoutPointTag.app2 = 1;
				    MoutPointTag.app1 = FLASH_INVALID_DATA;
				    printf("app 1 update failed\n");
		   }else if(MoutPointTag.running == 2){
			      MoutPointTag.app1 = 1;
				    MoutPointTag.app2 = FLASH_INVALID_DATA;
				    printf("app 1 update failed\n");
			 }else{
			     return;
			 }
			 MoutPointTag.running == FLASH_INVALID_DATA;	 
			 UART2_send(AckFail,sizeof(AckFail));
			 STMFLASH_Write(FLASH_CFG_MP_ADDR,&MoutPointTag.magic,4);
	 }else{
		   printf("reset factory mode\n");
		   MoutPointTag.magic = FLASH_CFG_MAGIC;
		   MoutPointTag.app1 = 1;
		   MoutPointTag.app2 = FLASH_INVALID_DATA;
		   MoutPointTag.running = FLASH_INVALID_DATA;
	     STMFLASH_Write(FLASH_CFG_MP_ADDR,&MoutPointTag.magic,4);
	 } 
}
//appxaddr:Ӧ�ó������ʼ��ַ
//appbuf:Ӧ�ó���CODE.
//appsize:Ӧ�ó����С(�ֽ�).
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u16 t;
	u16 i=0;
	u16 temp;
	u32 fwaddr=appxaddr;//��ǰд��ĵ�ַ
	u8 *dfu=appbuf;
	for(t=0;t<appsize;t+=2)
	{						    
		temp=(u16)dfu[1]<<8;
		temp+=(u16)dfu[0];	  
		dfu+=2;//ƫ��2���ֽ�
		iapbuf[i++]=temp;	    
		if(i == FLASH_PAGE_SIZE/2)
		{
			i=0;
			STMFLASH_Write(fwaddr,iapbuf,FLASH_PAGE_SIZE/2);	
			fwaddr+=FLASH_PAGE_SIZE;//ƫ2048  16bit.����Ҫ����2.
		}
	}
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);//������һЩ�����ֽ�д��ȥ.  
}

void iap_readback_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u8 data8[2048]={0};
	u16 data16[2048/2];
	u16 i=0,j=0,k=0;
	#define   RX_CNT  2048
	
	while( k<9 ){
	  STMFLASH_Read(appxaddr+k*2048,data16,RX_CNT/2);
		for(int i=0;i<RX_CNT/2;i++){
				data8[i*2] = data16[i] & 0xff;
				data8[i*2+1] = data16[i] >>8;
		}
		
		for(int i=0;i<RX_CNT;i++){
			if(data8[i] != usart2_rx_buf[i+k*2048]){
				printf("data8: 0x%x != usart2_rx_buf:0x%x, %d\n", data8[i],usart2_rx_buf[i+k*2048],i);
			}
		}
		k++;
	}
	STMFLASH_Read(appxaddr+8*2048,data16,426);
		for(int i=0;i<RX_CNT/2;i++){
				data8[i*2] = data16[i] & 0xff;
				data8[i*2+1] = data16[i] >>8;
		}

}
//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(u32 appxaddr)
{
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
		MSR_MSP(*(vu32*)appxaddr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		jump2app();									//��ת��APP.
	}
}		 


