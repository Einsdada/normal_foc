/**
 * @file    pid.h
 * @brief   PI 控制器（位置式，带积分/输出限幅），平台无关
 *
 * 用途：**外环** —— 速度环（spd_target → Iq 给定）、位置环（pos_target → 速度给定）。
 *       电流环不走这里：见 Algorithm/mc_current.c（dq 矢量控制器：
 *       PI + 前馈解耦 + 电压矢量限幅 + 抗饱和）。
 * 输出约定：输出是下一级给定，量纲由使用者定义（如速度环输出 A），限幅由使用者设置。
 * 积分限幅：积分项 ki·∫err 不超出输出限幅（防积分饱和 windup），ki=0 时纯比例。
 *
 * @note 积分的时间基准：PID_Calc 按 **调用次数** 累加（integral += err），不做 dt
 *       归一 → ki 隐含"每拍"量纲，随调用频率变化（同一 ki 在 20kHz 与 10kHz 下等效
 *       积分增益差 2 倍）。改控制频率/分频时必须同步重算 ki；要频率无关就显式乘 dt
 *       （电流环 mc_current 用的就是 dt 归一形式）。
 */
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  PI 控制器句柄
 */
typedef struct {
    float kp;           /**< 比例增益 */
    float ki;           /**< 积分增益 */
    float out_min;      /**< 输出下限（双极性，如 -0.3） */
    float out_max;      /**< 输出上限（如 +0.3） */
    float integral;     /**< 积分累加（内部状态，无需外部读写） */
    float out;          /**< 本次输出（内部状态） */
} PID_Handle_t;

/**
 * @brief  初始化 PI 控制器
 * @param  pid      句柄
 * @param  kp       比例增益
 * @param  ki       积分增益（0 = 纯比例）
 * @param  out_min  输出下限
 * @param  out_max  输出上限
 */
void PID_Init(PID_Handle_t *pid, float kp, float ki, float out_min, float out_max);

/**
 * @brief  执行一拍 PI 计算
 * @param  pid  句柄
 * @param  err  误差（给定 - 反馈）
 * @retval 限幅后的输出
 * @note   每个控制周期调用一次；参数（kp/ki/限幅）可在运行中直接改句柄字段
 */
float PID_Calc(PID_Handle_t *pid, float err);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
