#ifndef __MT6701_H__
#define __MT6701_H__

#include "stdint.h"
#include "main.h"       /* GPIO 引脚宏 */

/* =====================================================================
 * MT6701 磁编码器驱动统一头文件
 * 说明：
 *   - 芯片相关的宏定义统一在本文件管理
 *   - 对外提供 SSI / ABZ 两种接口模式的调用（实现见 mt6701_ssi.c / mt6701_abz.c）
 * ===================================================================== */

/* ============ 芯片协议常量 ============ */
#define MT6701_SSI_CPR   16384   /* SSI 14bit：每圈计数总数 */
#define MT6701_ABZ_CPR   4096    /* ABZ：编码器线数×4 四倍频（1024 线，每圈计数总数） */

/* ============ SSI 板级引脚配置（按硬件接线修改） ============ */
/* CSN：软件 GPIO 控制（PA15，CubeMX 配置为 GPIO_Output）
   硬件 NSS 的 CSN 与 CLK 同步、建立时间不足（MT6701 要求 TL>=100ns），已退回软件控制 */
#define MT6701_SSI_CSN_PORT   Encoder_CSN_GPIO_Port
#define MT6701_SSI_CSN_PIN    Encoder_CSN_Pin

#define MT6701_SSI_MODE_PORT  Encoder_mode_GPIO_Port
#define MT6701_SSI_MODE_PIN   Encoder_mode_Pin

/* ============ SSI 数据结构 ============ */
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
 * @brief  SSI 原始解析数据结构体
 * @note   24bit 帧
 */
typedef struct {
    Magnet_FieldStatus_e field_strength;   // 磁场状态
    uint16_t             angle;            // 14位角度
    uint8_t              push_button;      // 按键状态
    uint8_t              loss_of_track;    // 失磁标志
} Encoder_RawData_t;

/* ============ SSI 接口（实现：mt6701_ssi.c） ============ */
void    MT6701_Ssi_Init(void);
uint8_t MT6701_Ssi_Process_RawData(Encoder_RawData_t *data);

/* ============ ABZ 接口（实现：mt6701_abz.c） ============ */
void    MT6701_Abz_Init(void);
int32_t MT6701_Abz_GetCount(void);
void    MT6701_Abz_Clear(void);

/* ============ ABZ 分辨率配置（实现：mt6701_i2c_soft.c） ============ */
uint8_t MT6701_Abz_Configure(uint16_t ppr);   /* 软件 I2C 写 RAM 配置，返回 1=成功 */

#endif
