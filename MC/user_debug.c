#include "user_debug.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

// 发送缓冲区
static uint8_t uart_tx_buffer[TX_BUF_LEN];
// 用于DMA发送完成的标志
static volatile uint8_t uart_tx_busy = 0;

/* --------------------------------- 调试TX --------------------------------- */

/**
 * @brief 初始化（当前只发送：仅初始化串口，不开接收中断；
 *        接收命令后期开发）
 */
void Debug_Usart_Init(void)
{
    MX_USART1_UART_Init();
}

// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        uart_tx_busy = 0;
    }
}

// 基于DMA的串口printf函数（返回 0 = 串口忙，本次内容被丢弃，调用方可重试）
uint8_t uart_printf(const char *format, ...) {
    va_list args;
    uint16_t len;
    
    // 等待上一次DMA传输完成
    if (uart_tx_busy) return 0;
    
    // 格式化字符串到缓冲区
    va_start(args, format);
    len = vsnprintf((char *)uart_tx_buffer, TX_BUF_LEN, format, args);
    va_end(args);
    
    // 检查长度是否超出缓冲区
    if (len >= TX_BUF_LEN) {
        len = TX_BUF_LEN - 1;
    }
    
    // 标记DMA传输忙
    uart_tx_busy = 1;
    
    // 使用DMA发送数据
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, len);
    return 1;
}


static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void uart_send_justfloat(float *data, uint16_t count)
{
    if (uart_tx_busy) return; // 忙就丢掉，避免阻塞
    
    uint16_t len = 0;

    // 先拷贝浮点数组
    memcpy(&uart_tx_buffer[len], data, count * sizeof(float));
    len += count * sizeof(float);

    // 加上帧尾
    memcpy(&uart_tx_buffer[len], justfloat_tail, 4);
    len += 4;

    uart_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, len);
}


/* --------------------------------- 调试RX --------------------------------- */
/* 接收命令后期开发：届时启用 HAL_UART_Receive_IT 并注册
   HAL_UART_RxCpltCallback（单字节缓存 → 主循环解析） */



