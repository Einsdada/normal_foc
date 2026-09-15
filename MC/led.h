#ifndef __LED_H__
#define __LED_H__

#include "stdint.h"

void Led_Init(void);    /* 初始化：启动 TIM17_CH1 PWM */
void Led_Loop(void);    /* 呼吸效果，放在 1ms 定时中断里调用 */

#endif
