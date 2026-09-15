#ifndef __USER_DEBUG_H
#define __USER_DEBUG_H

#include "stdint.h"


void Debug_Usart_Init(void);


/* --------------------------------- 调试TX --------------------------------- */

#define TX_BUF_LEN 	128

// 函数声明
void uart_printf(const char *format, ...);
void uart_send_justfloat(float *data, uint16_t count);





#endif
