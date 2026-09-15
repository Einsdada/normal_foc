#include "user_debug.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

uint8_t rx_data;
// 发送缓冲区
static uint8_t uart_tx_buffer[TX_BUF_LEN];
// 用于DMA发送完成的标志
static volatile uint8_t uart_tx_busy = 0;

/* --------------------------------- 调试TX --------------------------------- */

/**
 * @brief 初始化
 * 
 */
void Debug_Usart_Init(void)
{
    MX_USART1_UART_Init();
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

// DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        uart_tx_busy = 0;
    }
}

// 基于DMA的串口printf函数
void uart_printf(const char *format, ...) {
    va_list args;
    uint16_t len;
    
    // 等待上一次DMA传输完成
    if (uart_tx_busy) return;
    
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



/* --------------------------------- 调试应用处理 --------------------------------- */

// uint8_t Debug_Justfloat(MC_Handle_t *handle)
// {
//     float values[9];
//     values[0] = handle->Target.position_rad;
//     values[1] = handle->monitor.position_rad;
//     values[2] = handle->Target.speed_rad_s;
//     values[3] = handle->monitor.speed_rad_s;
//     values[4] = handle->Target.torque;
//     values[5] = handle->monitor.torque;
//     values[6] = handle->monitor.angle_rad;
//     values[7] = 0.0f; // 保留字段
//     values[8] = 0.0f; // 保留字段

//     uart_send_justfloat(values, 9);

//     return 0;
// }



