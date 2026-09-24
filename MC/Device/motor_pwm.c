#include "motor_pwm.h"
#include "drv8311.h"    /* 设备层内部绑定 BSP PWM 接口 */

/* =====================================================================
 * 电机 PWM 输出设备层实现
 * - BSP（DRV8311）在 Init 内部绑定并启动三路 PWM
 * - SetDuty 未使能时只缓存目标占空比，Enable 后统一生效（防误输出）
 * - Disable 直接关断驱动器（sleep 低），电机无源
 * ===================================================================== */

/**
 * @brief  初始化电机 PWM：绑定 BSP 并启动三路 PWM 载波
 * @param  handle  电机 PWM 句柄
 * @note   BSP 在设备层内部绑定/启动（Drv8311_Pwm_Init，0% 占空比）
 *         初始未使能（需 MotorPwm_Enable 后才有输出）
 */
void MotorPwm_Init(MotorPwm_Handle_t *handle)
{
    handle->set_duty = Drv8311_Pwm_SetDuty;     /* 设备层内部绑定 BSP 占空比接口 */
    handle->duty[0]  = 0.0f;
    handle->duty[1]  = 0.0f;
    handle->duty[2]  = 0.0f;
    handle->enabled  = 0;

    Drv8311_Pwm_Init();                         /* 启动三路 PWM（0% 载波） */
}

/**
 * @brief  设置三相目标占空比
 * @param  handle  电机 PWM 句柄
 * @param  a,b,c   A/B/C 相占空比 0.0~1.0（归一化，超限截断）
 * @note   未使能时只缓存；已使能时立即输出到 BSP
 */
void MotorPwm_SetDuty(MotorPwm_Handle_t *handle, float a, float b, float c)
{
    if (a < 0.0f) a = 0.0f; else if (a > 1.0f) a = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    if (c < 0.0f) c = 0.0f; else if (c > 1.0f) c = 1.0f;

    handle->duty[0] = a;
    handle->duty[1] = b;
    handle->duty[2] = c;

    if (handle->enabled)
    {
        handle->set_duty(a, b, c);      /* 一次设置三相 */
    }
}

/**
 * @brief  使能电机输出：驱动器 sleep 高 + 恢复缓存占空比
 * @param  handle  电机 PWM 句柄
 */
void MotorPwm_Enable(MotorPwm_Handle_t *handle)
{
    Drv8311_Pwm_Enable();                       /* sleep 高，驱动器工作 */
    handle->set_duty(handle->duty[0], handle->duty[1], handle->duty[2]);   /* 恢复缓存占空比 */
    handle->enabled = 1;
}

/**
 * @brief  确保输出已使能（幂等：已使能则什么都不做）
 * @param  handle  电机 PWM 句柄
 * @retval 1 = 本次调用刚使能（即此前处于关断状态，调用方可据此做一次性初始化）
 *         0 = 本来就已使能
 * @note   每拍无条件调用安全：未使能时 MotorPwm_SetDuty 只缓存不输出（防误输出），
 *         各模式从停止切入时不必自己判断使能状态，统一由本接口兜底
 */
uint8_t MotorPwm_EnsureEnabled(MotorPwm_Handle_t *handle)
{
    if (handle->enabled != 0u) {
        return 0u;
    }
    MotorPwm_Enable(handle);
    return 1u;
}

/**
 * @brief  关断电机输出：驱动器 sleep 低，三相关断
 * @param  handle  电机 PWM 句柄
 */
void MotorPwm_Disable(MotorPwm_Handle_t *handle)
{
    handle->enabled = 0;
    Drv8311_Pwm_Disable();                      /* sleep 低，三相关断 */
}

/**
 * @brief  停机：占空比清零 + 关断输出（设备层定义的安全态）
 * @param  handle  电机 PWM 句柄
 * @note   占空比必须先清零再关断：Disable 只拉低 sleep，缓存占空比会保留到下次
 *         使能，清零可保证下次 Enable 的瞬间不会先输出上一次的残留占空比；
 *         幂等，停止模式每拍调用安全
 */
void MotorPwm_Stop(MotorPwm_Handle_t *handle)
{
    MotorPwm_SetDuty(handle, 0.0f, 0.0f, 0.0f);
    MotorPwm_Disable(handle);
}

/**
 * @brief  读驱动器故障（nFAULT）
 * @param  handle  电机 PWM 句柄
 * @retval 1 = 有故障（DRV8311 输出级被关断：过流/欠压/过温等）
 * @note   用于排查"给了电压却不走电流"；引脚配置在 BSP 内部（带内部上拉的输入）
 */
uint8_t MotorPwm_FaultRead(MotorPwm_Handle_t *handle)
{
    (void)handle;
    return Drv8311_FaultRead();
}
