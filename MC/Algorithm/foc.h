/**
 * @file    foc.h
 * @brief   FOC 算法层（MC/Algorithm）
 *
 * 与平台无关的 FOC 核心算法：SVPWM 计算等。
 * 输入输出约定（全归一化）：
 *  - 电角度由调用方传入 sin/cos（避免内部重复三角函数开销）；
 *  - 电压 Ud/Uq 为归一化调制比，范围 [-1, 1]：
 *      1.0 = 相电压峰值 = Udc/2（等幅值约定，线性区边界；SVPWM 零序注入后
 *      实际可线性调制到 1.1547，超出 1.0 部分削顶，不建议依赖）；
 *  - SVPWM 输出 u/v/w 为三相桥臂占空比，范围 [0, 1]：
 *      1 = 上桥臂全通（+Udc/2），0 = 下桥臂全通（-Udc/2），0.5 = 中点（零平均电压）
 *     PWM 层可直接写入 MotorPwm_SetDuty，无需再换算
 */
#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SVPWM 计算：逆 Park + 反 Clarke + 零序注入（中心对齐，全部归一化）
 * @param  Ud     d 轴电压调制比 [-1, 1]（1.0 = 相电压峰值 Udc/2）
 * @param  Uq     q 轴电压调制比 [-1, 1]
 * @param  sin    电角度正弦 sin(θe)
 * @param  cos    电角度余弦 cos(θe)
 * @param  u      输出 A 相桥臂占空比 [0, 1]（0.5 = 零电压）
 * @param  v      输出 B 相桥臂占空比 [0, 1]
 * @param  w      输出 C 相桥臂占空比 [0, 1]
 * @note   反 Clarke 为等幅值：A 相 = α 轴（Va=uα, Vb=(−uα+√3·uβ)/2, Vc=(−uα−√3·uβ)/2）；
 *         min-max 零序注入（三相占空比之和恒为 3×0.5，中心对齐）；过调制削顶
 */
void MC_SVPWM_Calc(float Ud, float Uq, float sin, float cos,
                   float *u, float *v, float *w);

/**
 * @brief  Clarke 变换（等幅值）：三相电流 → αβ 轴电流
 * @param  ia, ib, ic  三相电流（A）
 * @param  i_alpha     α 轴电流（= ia）
 * @param  i_beta      β 轴电流 = ia×(1/√3) + ib×(2/√3)
 * @note   三相平衡假设（星形绕组，无中线），ic 参数仅作占位；
 *         β 轴为两项常量系数乘加（无除法，可 FMA 融合）
 */
void MC_Clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta);

/**
 * @brief  Park 变换：αβ 轴电流 → dq 轴电流
 * @param  i_alpha  α 轴电流
 * @param  i_beta   β 轴电流
 * @param  sin,cos  电角度 sin(θe)/cos(θe)（调用方查表传入）
 * @param  i_d      d 轴电流输出
 * @param  i_q      q 轴电流输出
 */
void MC_Park(float i_alpha, float i_beta, float sin, float cos,
             float *i_d, float *i_q);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H__ */
