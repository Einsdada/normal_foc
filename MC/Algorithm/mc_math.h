/**
 * @file    mc_math.h
 * @brief   算法层数学工具（MC/Algorithm）：查表法正余弦
 *
 * 用整周期正弦表 + 线性插值计算 sin/cos，替代标准库 sinf/cosf：
 *  - 无 libm 依赖（不链接数学库），运算量固定、确定性好；
 *  - 256 点表（1KB Flash），线性插值最大误差 ≈ 7.5e-5（电压调制比 0~1 下误差 < 0.01%）；
 *  - 输入弧度角。注意：角度应保持在 [0, 2π) 内（调用方做周期归约），
 *    过大的角度值会因 float 尾数精度损失导致查表失效。
 */
#ifndef __MC_MATH_H__
#define __MC_MATH_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  查表计算 sin(angle)
 * @param  angle  角度 (rad)，建议 [0, 2π) 内
 * @retval 正弦值 [-1, 1]
 */
float MC_Math_Sin(float angle);

/**
 * @brief  查表计算 cos(angle)（sin(angle + π/2)）
 * @param  angle  角度 (rad)，建议 [0, 2π) 内
 * @retval 余弦值 [-1, 1]
 */
float MC_Math_Cos(float angle);

/**
 * @brief  一次算出 sin/cos（FOC 每拍都要同时用，比分别调用省一次角度归一化+索引计算）
 * @param  angle  角度 (rad)，建议 [0, 2π) 内
 * @param  s, c   输出：sin/cos [-1, 1]
 * @note   与 MC_Math_Sin(angle)、MC_Math_Cos(angle) 数值一致：cos 用同一个
 *         小数插值系数 frac，只把表索引 +N/4（省掉 (angle+π/2) 那次归一化），
 *         差异仅在 float 舍入量级（远小于插值误差 7.5e-5）
 */
void MC_Math_SinCos(float angle, float *s, float *c);

#ifdef __cplusplus
}
#endif

#endif /* __MC_MATH_H__ */
