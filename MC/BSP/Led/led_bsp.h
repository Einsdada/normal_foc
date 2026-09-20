#ifndef __LED_BSP_H__
#define __LED_BSP_H__

#include "stdint.h"
#include "main.h"       /* GPIO 端口/引脚宏 */
#include "tim.h"        /* TIM_HandleTypeDef 与 TIM_CHANNEL_* */

/* =====================================================================
 * LED 板级驱动统一头文件（BSP 层）
 * - 驱动方式由 LED_BSP_USE_PWM 选择（改这里一个宏即切换）
 * - 通道配置集中在本文件配置区：增加/修改 LED 只改这里
 * - 实现分两个文件（都参与编译，宏决定哪个提供实现）：
 *     led_bsp_pwm.c   PWM 方式：定时器输出，亮度 0~LED_BSP_DUTY_MAX 连续可调
 *     led_bsp_gpio.c  GPIO 方式：普通 IO，>0 亮 / 0 灭
 * ===================================================================== */

#define LED_BSP_USE_PWM    1       /* 1=PWM 驱动，0=GPIO 驱动（按硬件修改） */

#define LED_BSP_DUTY_MAX    1000   /* 占空比满量程（PWM 通道 = ARR） */
#define LED_BSP_ACTIVE_LOW  1      /* 1=LED 低电平点亮（IO 低=亮），0=高电平点亮（按硬件修改） */

#if LED_BSP_USE_PWM
/* ============ PWM 方式通道配置区（每通道 2 行） ============ */
#define LED_BSP_CH_COUNT    1

#define LED_BSP_CH0_TIM     &htim17
#define LED_BSP_CH0_CH      TIM_CHANNEL_1

/* 增加通道示例：
#define LED_BSP_CH1_TIM     &htim1
#define LED_BSP_CH1_CH      TIM_CHANNEL_1
*/
#else
/* ============ GPIO 方式通道配置区（每通道 3 行） ============ */
#define LED_BSP_CH_COUNT    1

#define LED_BSP_CH0_PORT    GPIOA
#define LED_BSP_CH0_PIN     GPIO_PIN_8
#define LED_BSP_CH0_ACTIVE  1          /* 1=高电平亮，0=低电平亮 */

/* 增加通道示例：
#define LED_BSP_CH1_PORT    GPIOB
#define LED_BSP_CH1_PIN     GPIO_PIN_5
#define LED_BSP_CH1_ACTIVE  0
*/
#endif

/* ============ 对外接口（实现：led_bsp_pwm.c / led_bsp_gpio.c） ============ */
void Led_Bsp_Init(void);                            /* 初始化所有通道（初始熄灭） */
void Led_Bsp_SetDuty(uint8_t ch, uint16_t duty);    /* 通道 0~(CH_COUNT-1)，duty 0~DUTY_MAX */

#endif
