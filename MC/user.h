#ifndef __USER_H__
#define __USER_H__

#include "main.h"   /* GPIOB / GPIO_PIN_x / TEXTx_Pin */

void User_Init(void);

void User_Loop(void);

/* =====================================================================
 * 调试观测 IO（PB3/PB4/PB5 = main.h 的 TEXT3/TEXT2/TEXT1）
 * 用法：TEST1(1) 拉高 / TEST1(0) 拉低，寄存器 BSRR 直写
 * ===================================================================== */
#define TEST_IO_BSRR(pin, v)  do {              \
    if (v)  GPIOB->BSRR = (pin);                \
    else    GPIOB->BSRR = (uint32_t)(pin) << 16U; \
} while (0)

#define TEST1(v)   TEST_IO_BSRR(GPIO_PIN_5, (v))   /* PB5 */
#define TEST2(v)   TEST_IO_BSRR(GPIO_PIN_4, (v))   /* PB4 */
#define TEST3(v)   TEST_IO_BSRR(GPIO_PIN_3, (v))   /* PB3 */

#endif
