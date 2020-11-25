#ifndef __PI5008K_FUNC_H__
#define __PI5008K_FUNC_H__



enum Rem_Key_Enum {
  REM_DIR_UP = 0,
  REM_DIR_UP_LONG,
  REM_DIR_DOWN,
  REM_DIR_DOWN_LONG,
  REM_DIR_LEFT,
  REM_DIR_LEFT_LONG,
  REM_DIR_RIGHT,
  REM_DIR_RIGHT_LONG,
  REM_DIR_CENTER,
  REM_DIR_CENTER_LONG,
  REM_MENU,
  REM_MENU_LONG,
  REM_NAVI,
  REM_NAVI_LONG,
  REM_MAX, 
};


#define PI5008K_USART_BUFFER_SIZE 50


void PI5008K_Init(void);

void PI5008K_UartCmdProcessing(void);
void PI5008K_Uart_Con_Remote_Cmd(unsigned int dir);
void PI5008K_Uart_Reg_Single_Write(unsigned int  addr,unsigned int data);
int PI5008K_5Dir_Key_Read(void);
int PI5008K_5Dir_Center_Key_Led(void);
void UART1_PRINTF(const char *format, ...);

#endif /* __PI5008K_FUNC_H__ */

