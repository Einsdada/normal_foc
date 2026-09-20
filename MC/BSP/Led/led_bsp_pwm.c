#include "led_bsp.h"

/* =====================================================================
 * LED 板级驱动：PWM 方式
 * 仅在 LED_BSP_USE_PWM = 1 时编译（提供 Led_Bsp_Init / Led_Bsp_SetDuty）
 * 通道表由 led_bsp.h 配置区生成，增加通道只改配置区
 * ===================================================================== */

#if LED_BSP_USE_PWM

typedef struct {
    TIM_HandleTypeDef *tim;    /* 定时器句柄（如 &htim17） */
    uint32_t          ch;      /* 定时器输出通道（TIM_CHANNEL_x） */
} Led_BspPwmCh_t;

/* ---------- 通道表（由 led_bsp.h 配置区生成，与 LED_BSP_CH_COUNT 一致） ---------- */
static const Led_BspPwmCh_t pwm_tbl[LED_BSP_CH_COUNT] = {
    { LED_BSP_CH0_TIM, LED_BSP_CH0_CH },
#if LED_BSP_CH_COUNT > 1
    { LED_BSP_CH1_TIM, LED_BSP_CH1_CH },
#endif
#if LED_BSP_CH_COUNT > 2
    { LED_BSP_CH2_TIM, LED_BSP_CH2_CH },
#endif
#if LED_BSP_CH_COUNT > 3
    { LED_BSP_CH3_TIM, LED_BSP_CH3_CH },
#endif
#if LED_BSP_CH_COUNT > 4
    { LED_BSP_CH4_TIM, LED_BSP_CH4_CH },
#endif
#if LED_BSP_CH_COUNT > 5
    { LED_BSP_CH5_TIM, LED_BSP_CH5_CH },
#endif
#if LED_BSP_CH_COUNT > 6
    { LED_BSP_CH6_TIM, LED_BSP_CH6_CH },
#endif
#if LED_BSP_CH_COUNT > 7
    { LED_BSP_CH7_TIM, LED_BSP_CH7_CH },
#endif
};

/**
 * @brief  初始化全部 PWM 通道：先置占空比 0（熄灭）再启动 PWM 输出
 * @note   TIM 的参数（预分频/ARR）在 CubeMX 生成的 tim.c 中配置
 *         低电平点亮时"熄灭"= CCR 满（IO 恒高），由 LED_BSP_ACTIVE_LOW 处理
 */
void Led_Bsp_Init(void)
{
    for (uint8_t i = 0; i < LED_BSP_CH_COUNT; i++)
    {
#if LED_BSP_ACTIVE_LOW
        __HAL_TIM_SET_COMPARE(pwm_tbl[i].tim, pwm_tbl[i].ch, LED_BSP_DUTY_MAX);   /* 初始熄灭（反相=CCR 满） */
#else
        __HAL_TIM_SET_COMPARE(pwm_tbl[i].tim, pwm_tbl[i].ch, 0);
#endif
        HAL_TIM_PWM_Start(pwm_tbl[i].tim, pwm_tbl[i].ch);
    }
}

/**
 * @brief  设置指定通道亮度（占空比）
 * @param  ch    通道号 0~(LED_BSP_CH_COUNT-1)，越界自动忽略
 * @param  duty  占空比 0~LED_BSP_DUTY_MAX（=ARR），超限自动截断
 * @note   低电平点亮时内部反相（duty → DUTY_MAX - duty），对上层透明
 */
void Led_Bsp_SetDuty(uint8_t ch, uint16_t duty)
{
    if (ch >= LED_BSP_CH_COUNT) return;
    if (duty > LED_BSP_DUTY_MAX) duty = LED_BSP_DUTY_MAX;
#if LED_BSP_ACTIVE_LOW
    duty = LED_BSP_DUTY_MAX - duty;   /* 反相：上层 duty 大=亮，IO 低电平时间多 */
#endif
    __HAL_TIM_SET_COMPARE(pwm_tbl[ch].tim, pwm_tbl[ch].ch, duty);
}

#endif /* LED_BSP_USE_PWM */
