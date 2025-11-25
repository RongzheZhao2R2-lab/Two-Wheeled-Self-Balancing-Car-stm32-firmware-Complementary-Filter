

#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"


extern unsigned short SoftTimer[5];

void SoftTimerCountDown(void);
void SysTick_Init(void);


#endif /* __SYSTICK_H */

