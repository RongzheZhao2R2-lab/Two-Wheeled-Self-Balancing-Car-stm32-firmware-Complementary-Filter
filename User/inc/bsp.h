
#ifndef _BSP_H
#define _BSP_H

#include "stm32f10x_gpio.h"






#define LED1On		GPIO_ResetBits(GPIOA,GPIO_Pin_3)
#define LED1Off		GPIO_SetBits(GPIOA,GPIO_Pin_3)

#define LED2On		GPIO_ResetBits(GPIOA,GPIO_Pin_12)
#define LED2Off		GPIO_SetBits(GPIOA,GPIO_Pin_12)




void LEDToggle(void);

void BspInit(void);




#endif

