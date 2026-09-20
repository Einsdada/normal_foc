#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

#include "stdint.h"

/* =====================================================================
 * 电机 PWM 输出设备层
 * - 三相 PWM 占空比控制（三线输出，DRV8311 集成门驱动内部生成互补与死区）
 * - 底层 BSP（DRV8311 PWM）在设备层内部绑定，上层无感知
 * - 占空比接口统一归一化：0.0~1.0（映射到 ARR 由 BSP 完成）
 * - 安全控制：Disable 关断驱动器（sleep 低），Enable 恢复输出
 * 实现：motor_pwm.c
 * ===================================================================== */

typedef struct {
    void  (*set_duty)(float a, float b, float c);   /* BSP 三相占空比接口（设备层内部绑定，0.0~1.0） */
    float duty[3];      /* 三相占空比缓存（A/B/C，0.0~1.0） */
    uint8_t enabled;    /* 输出是否已使能 */
} MotorPwm_Handle_t;

void MotorPwm_Init(MotorPwm_Handle_t *handle);                                  /* 绑定 BSP 并启动三路 PWM（0% 占空比） */
void MotorPwm_SetDuty(MotorPwm_Handle_t *handle, float a, float b, float c);    /* 设置三相占空比（0.0~1.0，已使能时立即输出） */
void MotorPwm_Enable(MotorPwm_Handle_t *handle);                                /* 使能输出（sleep 高 + 恢复缓存占空比） */
void MotorPwm_Disable(MotorPwm_Handle_t *handle);                               /* 关断输出（sleep 低，三相关断） */

#endif
