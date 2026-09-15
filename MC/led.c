#include "led.h"
#include "tim.h"

/* 呼吸灯：LED 接 PA7，由 TIM17_CH1 输出 PWM
 * TIM17: 170MHz/170/1000 = 1kHz，比较值 0~1000 对应占空比 0~100%
 * 每 20ms 走一步、每步 5/1000，0→1000→0 一圈约 8 秒 */

#define LED_LEVEL_MAX   1000        /* 最大比较值 = ARR */
#define LED_STEP_MS     20          /* 每 20ms 走一步 */
#define LED_STEP_VAL    5           /* 每步亮度增量 */

static uint16_t level = 0;          /* 当前亮度 */
static uint8_t  dir   = 1;          /* 1=变亮, 0=变暗 */
static uint8_t  cnt   = 0;          /* 毫秒计数 */

void Led_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void Led_Loop(void)
{
    if (++cnt < LED_STEP_MS) return;
    cnt = 0;

    if (dir) {                          /* 渐亮 */
        level += LED_STEP_VAL;
        if (level >= LED_LEVEL_MAX) { level = LED_LEVEL_MAX; dir = 0; }
    } else {                            /* 渐暗 */
        if (level >= LED_STEP_VAL) level -= LED_STEP_VAL;
        else                       level = 0;
        if (level == 0) dir = 1;
    }

    __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, level);
}
