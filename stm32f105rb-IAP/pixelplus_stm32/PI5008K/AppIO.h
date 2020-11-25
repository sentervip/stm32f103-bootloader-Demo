#ifndef __APP_IO_H_
#define __APP_IO_H_

//led pin define
#define RCC_LED1 RCC_APB2Periph_GPIOA
#define PORT_LED1 GPIOA
#define LED1_PIN GPIO_Pin_15


#define RCC_LED2 RCC_APB2Periph_GPIOC
#define PORT_LED2 GPIOC
#define LED2_PIN GPIO_Pin_10

#define RCC_LED3 RCC_APB2Periph_GPIOC
#define PORT_LED3 GPIOC
#define LED3_PIN GPIO_Pin_11

#define RCC_LED4 RCC_APB2Periph_GPIOC
#define PORT_LED4 GPIOC
#define LED4_PIN GPIO_Pin_12


//swtich pin define
#define RCC_KEY1 RCC_APB2Periph_GPIOB
#define PORT_KEY1 GPIOB
#define KEY1_PIN GPIO_Pin_12

#define RCC_KEY2 RCC_APB2Periph_GPIOB
#define PORT_KEY2 GPIOB
#define KEY2_PIN GPIO_Pin_13

#define RCC_KEY3 RCC_APB2Periph_GPIOB
#define PORT_KEY3 GPIOB
#define KEY3_PIN GPIO_Pin_14

#define RCC_KEY4 RCC_APB2Periph_GPIOB
#define PORT_KEY4 GPIOB
#define KEY4_PIN GPIO_Pin_15

#define LED1_ON() GPIO_ResetBits(PORT_LED1,LED1_PIN)
#define LED1_OFF() GPIO_SetBits(PORT_LED1,LED1_PIN)

#define LED2_ON() GPIO_ResetBits(PORT_LED2,LED2_PIN)
#define LED2_OFF() GPIO_SetBits(PORT_LED2,LED2_PIN)

#define LED3_ON() GPIO_ResetBits(PORT_LED3,LED3_PIN)
#define LED3_OFF() GPIO_SetBits(PORT_LED3,LED3_PIN)

#define LED4_ON() GPIO_ResetBits(PORT_LED4,LED4_PIN)
#define LED4_OFF() GPIO_SetBits(PORT_LED4,LED4_PIN)


#define READ_SWTICH_1() (!GPIO_ReadInputDataBit(PORT_KEY1,KEY1_PIN))
#define READ_SWTICH_2() (!GPIO_ReadInputDataBit(PORT_KEY2,KEY2_PIN))
#define READ_SWTICH_3() (!GPIO_ReadInputDataBit(PORT_KEY3,KEY3_PIN))
#define READ_SWTICH_4() (!GPIO_ReadInputDataBit(PORT_KEY4,KEY4_PIN))
void GPIO_Configuration(void);


#endif
