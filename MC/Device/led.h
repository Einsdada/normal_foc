#ifndef __LED_H__
#define __LED_H__

#include "stdint.h"

/* =====================================================================
 * LED 设备层（单色 LED，多实例）
 * - 设备层只负责"点亮方式"：常亮 / 闪烁 / 呼吸、亮度、波形、伽马校正
 * - 底层 BSP 在设备层内部绑定（Led_Init 时自动链接 BSP 亮度函数）：
 *   驱动方式是 PWM 还是 GPIO 由 BSP 决定，设备层与上层均无感知
 * - 各运行模式参数独立配置（每个模式一个设置接口）：
 *     常亮：亮度
 *     闪烁：亮度 + 周期 + 点亮时长占比
 *     呼吸：峰值 / 谷值亮度 + 周期 + 波形
 * - Led_Update 注入句柄与调用频率 freq（Hz），便于任意定时源驱动
 * - 亮度 0~255 经伽马校正映射到占空比（人眼感知线性化）
 * 实现：led.c
 * ===================================================================== */

typedef enum {
    LED_MODE_OFF = 0,   /* 熄灭 */
    LED_MODE_ON,        /* 常亮 */
    LED_MODE_BLINK,     /* 闪烁：周期内亮/灭按占比分配 */
    LED_MODE_BREATHE,   /* 呼吸：三角波/正弦波，min~max 渐变 */
} Led_Mode_e;

typedef enum {
    LED_WAVE_TRIANGLE = 0,  /* 三角波：线性渐亮渐暗 */
    LED_WAVE_SINE,          /* 正弦波：平滑呼吸 */
} Led_Wave_e;

/* BSP 亮度设置接口（函数指针）：ch=通道号，duty=占空比 0~LED_BSP_DUTY_MAX
   由设备层在 Led_Init 时绑定为 Led_Bsp_SetDuty */
typedef void (*Led_SetBright_t)(uint8_t ch, uint16_t duty);

typedef struct {
    uint8_t         ch;         /* BSP 通道号（Led_Init 注入） */
    Led_SetBright_t set_bright; /* BSP 亮度设置函数（设备层内部绑定，勿改动） */
    uint8_t         mode;       /* 运行模式 Led_Mode_e（内部） */
    uint8_t         bright;     /* 常亮亮度 / 闪烁亮电平 0~255 */
    uint8_t         bright_max; /* 呼吸峰值亮度 0~255 */
    uint8_t         bright_min; /* 呼吸谷值亮度 0~255（须 ≤ bright_max） */
    uint8_t         duty;       /* 闪烁点亮时长占比 0~100（%） */
    uint16_t        period_ms;  /* 闪烁/呼吸周期（ms，一个完整周期） */
    uint8_t         wave;       /* 呼吸波形 Led_Wave_e */
    uint32_t        cnt_ms;     /* 周期内时间计数（内部） */
} Led_Handle_t;

void Led_Init(Led_Handle_t *handle, uint8_t ch);                    /* 绑定 BSP 通道（内部链接 BSP 亮度函数）并初始化为熄灭 */
void Led_Update(Led_Handle_t *handle, uint16_t freq);               /* 周期调用：注入句柄 + 调用频率 Hz */

void Led_SetOn(Led_Handle_t *handle, uint8_t bright);               /* 常亮：指定亮度 */
void Led_SetBlink(Led_Handle_t *handle, uint8_t bright,             /* 闪烁：亮度 + 周期 + 点亮占比 */
                  uint16_t period_ms, uint8_t duty);
void Led_SetBreathe(Led_Handle_t *handle, uint8_t bright_max,       /* 呼吸：峰值/谷值亮度 + 周期 + 波形 */
                    uint8_t bright_min, uint16_t period_ms, uint8_t wave);
void Led_Off(Led_Handle_t *handle);                                 /* 熄灭 */

#endif
