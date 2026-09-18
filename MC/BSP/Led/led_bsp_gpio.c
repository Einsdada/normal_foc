#include "led_bsp.h"

/* =====================================================================
 * LED 板级驱动：GPIO 方式
 * 仅在 LED_BSP_USE_PWM = 0 时编译（提供 Led_Bsp_Init / Led_Bsp_SetDuty）
 * 通道表由 led_bsp.h 配置区生成，增加通道只改配置区
 * ===================================================================== */

#if !LED_BSP_USE_PWM

typedef struct {
    GPIO_TypeDef *port;    /* GPIO 端口（如 GPIOA） */
    uint16_t      pin;     /* GPIO 引脚（GPIO_PIN_x） */
    uint8_t       active;  /* 有效电平：1=高电平亮，0=低电平亮 */
} Led_BspGpioCh_t;

/* ---------- 通道表（由 led_bsp.h 配置区生成，与 LED_BSP_CH_COUNT 一致） ---------- */
static const Led_BspGpioCh_t gpio_tbl[LED_BSP_CH_COUNT] = {
    { LED_BSP_CH0_PORT, LED_BSP_CH0_PIN, LED_BSP_CH0_ACTIVE },
#if LED_BSP_CH_COUNT > 1
    { LED_BSP_CH1_PORT, LED_BSP_CH1_PIN, LED_BSP_CH1_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 2
    { LED_BSP_CH2_PORT, LED_BSP_CH2_PIN, LED_BSP_CH2_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 3
    { LED_BSP_CH3_PORT, LED_BSP_CH3_PIN, LED_BSP_CH3_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 4
    { LED_BSP_CH4_PORT, LED_BSP_CH4_PIN, LED_BSP_CH4_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 5
    { LED_BSP_CH5_PORT, LED_BSP_CH5_PIN, LED_BSP_CH5_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 6
    { LED_BSP_CH6_PORT, LED_BSP_CH6_PIN, LED_BSP_CH6_ACTIVE },
#endif
#if LED_BSP_CH_COUNT > 7
    { LED_BSP_CH7_PORT, LED_BSP_CH7_PIN, LED_BSP_CH7_ACTIVE },
#endif
};

/**
 * @brief  初始化全部 GPIO 通道（全部熄灭）
 * @note   引脚方向/初始电平由 CubeMX 生成的 gpio.c 配置，本处只置输出
 */
void Led_Bsp_Init(void)
{
    for (uint8_t i = 0; i < LED_BSP_CH_COUNT; i++)
    {
        Led_Bsp_SetDuty(i, 0);      /* 初始熄灭 */
    }
}

/**
 * @brief  设置指定通道亮度（GPIO 方式只有亮/灭两态）
 * @param  ch    通道号 0~(LED_BSP_CH_COUNT-1)，越界自动忽略
 * @param  duty  占空比 0~LED_BSP_DUTY_MAX：>0 点亮，=0 熄灭
 * @note   使用 BSRR 直写（置位/复位各一位），无读改写，速度最快
 */
void Led_Bsp_SetDuty(uint8_t ch, uint16_t duty)
{
    uint8_t on;
    if (ch >= LED_BSP_CH_COUNT) return;

    on = (duty > 0) ? 1U : 0U;
    if (gpio_tbl[ch].active) {
        if (on) gpio_tbl[ch].port->BSRR = gpio_tbl[ch].pin;
        else    gpio_tbl[ch].port->BSRR = (uint32_t)gpio_tbl[ch].pin << 16U;
    } else {
        if (on) gpio_tbl[ch].port->BSRR = (uint32_t)gpio_tbl[ch].pin << 16U;
        else    gpio_tbl[ch].port->BSRR = gpio_tbl[ch].pin;
    }
}

#endif /* !LED_BSP_USE_PWM */
