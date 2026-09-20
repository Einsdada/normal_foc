#include "drv8311.h"
#include "main.h"       /* Motor_sleep_Pin */
#include "tim.h"        /* htim1 与 TIM_CHANNEL_* */

/* =====================================================================
 * DRV8311 PWM 输出实现（三线：TIM1 CH1/CH2/CH3 → PA8/PA9/PA10）
 * - 三路占空比独立设置，接口统一归一化 0.0~1.0
 * - 占空比内部映射到 TIM1 ARR（4250，20kHz 中心对齐）
 * - 驱动器使能/关断走 sleep 引脚（PA11）：高=工作，低=三相关断
 * ===================================================================== */

/* ---------- 设置三相占空比（归一化 0.0~1.0，直接展开写三路，无中间层） ---------- */
void Drv8311_Pwm_SetDuty(float a, float b, float c)
{
    uint32_t cmp;

    /* A 相 → CCR1 */
    if (a <= 0.0f)      cmp = 0;
    else if (a >= 1.0f) cmp = DRV8311_PWM_ARR;
    else                cmp = (uint32_t)(a * DRV8311_PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp);

    /* B 相 → CCR2 */
    if (b <= 0.0f)      cmp = 0;
    else if (b >= 1.0f) cmp = DRV8311_PWM_ARR;
    else                cmp = (uint32_t)(b * DRV8311_PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp);

    /* C 相 → CCR3 */
    if (c <= 0.0f)      cmp = 0;
    else if (c >= 1.0f) cmp = DRV8311_PWM_ARR;
    else                cmp = (uint32_t)(c * DRV8311_PWM_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp);
}

/* ---------- 启动三路 PWM（0% 占空比载波） ---------- */
void Drv8311_Pwm_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8 → DRV8311 A 相输入 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* PA9 → B 相 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* PA10 → C 相 */
    /* 注：不输出互补 N（DRV8311 集成门驱动，内部生成互补与死区） */
}

/* ---------- 使能/关断驱动器输出 ---------- */
void Drv8311_Pwm_Enable(void)
{
    Motor_sleep_GPIO_Port->BSRR = Motor_sleep_Pin;      /* sleep 高，驱动器工作 */
}

void Drv8311_Pwm_Disable(void)
{
    Motor_sleep_GPIO_Port->BSRR = (uint32_t)Motor_sleep_Pin << 16U;   /* sleep 低，三相关断 */
}
