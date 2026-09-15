#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stdint.h"

/**
 * @brief  定义磁场状态
 */
typedef enum {
    MAGNET_FIELD_OK       = 0,   // 磁场正常
    MAGNET_FIELD_HIGH     = 1,   // 磁场过强
    MAGNET_FIELD_LOW      = 2,   // 磁场过弱
    MAGNET_FIELD_INVALID  = 3,   // 无定义
} Magnet_FieldStatus_e;

/**
 * @brief  原始解析数据结构体
 * @note   24bit
 */
typedef struct {
    Magnet_FieldStatus_e field_strength; // 磁场状态
    uint16_t    angle;          // 14位角度
    uint8_t     push_button;    // 按键状态
    uint8_t     loss_of_track;  // 失磁标志
} Encoder_RawData_t;

void Encoder_HW_Init(void);
uint8_t Encoder_Process_RawData(Encoder_RawData_t *data);


#endif
