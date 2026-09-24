#ifndef __MOTOR_PWM_H__
#define __MOTOR_PWM_H__

#include "stdint.h"

/* =====================================================================
 * 电机 PWM 输出设备层
 * - 三相 PWM 占空比控制（三线输出，DRV8311 集成门驱动内部生成互补与死区）
 * - 底层 BSP（DRV8311 PWM）在设备层内部绑定，上层无感知
 * - 占空比接口统一归一化：0.0~1.0（映射到 ARR 由 BSP 完成）
 * - 安全控制：Disable 关断驱动器（sleep 低），Enable 恢复输出；
 *               EnsureEnabled 幂等使能（返回是否刚使能），Stop 一键回安全态
 *               （占空比清零 + 关断）——"未使能则使能""停机"这类动作属于设备自身
 *               状态，统一封装在本层，控制层不再直接读写 enabled 字段
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
uint8_t MotorPwm_EnsureEnabled(MotorPwm_Handle_t *handle);                      /* 未使能则使能；返回 1 = 本次刚使能（从关断切入） */
void MotorPwm_Disable(MotorPwm_Handle_t *handle);                               /* 关断输出（sleep 低，三相关断） */
void MotorPwm_Stop(MotorPwm_Handle_t *handle);                                  /* 停机：占空比清零 + 关断输出（安全态，可重复调用） */
uint8_t MotorPwm_FaultRead(MotorPwm_Handle_t *handle);                          /* 读驱动器 nFAULT：1 = 有故障（输出级被关断） */

#endif
