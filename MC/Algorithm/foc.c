/**
 * @file    foc.c
 * @brief   FOC 算法层实现：SVPWM 计算
 *
 * 流程：
 *  1. 逆 Park 变换：dq 轴电压 → αβ 轴电压
 *       Uα = Ud·cosθ − Uq·sinθ
 *       Uβ = Ud·sinθ + Uq·cosθ
 *  2. SVPWM 三相中间量（等效桥臂电压，幅值以 Udc/2 归一化）
 *       U1 = Uβ
 *       U2 = (−Uβ + √3·Uα) / 2
 *       U3 = (−Uβ − √3·Uα) / 2
 *  3. 零序注入（min-max 中点偏移）：使三相中点落在调制范围内，
 *     等效 SVPWM 最大不失真调制比 2/√3 ≈ 1.1547
 *       offset = −(max(U1,U2,U3) + min(U1,U2,U3)) / 2
 *       u = U1 + offset，v = U2 + offset，w = U3 + offset
 *  4. 限幅到线性调制区 [-1, 1]
 */
#include "foc.h"

#define FOC_SQRT3       1.7320508075688772f   /* √3 */
#define FOC_INV_SQRT3   0.5773502691896258f   /* 1/√3：预计算常量，避免运行时除法 */
#define FOC_2_INV_SQRT3 1.1547005383792515f   /* 2/√3：同上 */

void MC_SVPWM_Calc(float Ud, float Uq, float sin, float cos,
                   float *u, float *v, float *w)
{
    float ua, ub;           /* αβ 轴电压 */
    float u1, u2, u3;       /* 三相反向电压中间量 */
    float umax, umin;       /* 中间量极值（零序注入） */
    float offset;           /* 中点偏移（零序分量） */

    /* ---- 1. 逆 Park 变换：输入 Ud/Uq 为调制比（[-1,1]，1.0 = 相电压峰值 Udc/2） ---- */
    ua =  Ud * cos - Uq * sin;      /* α 轴电压 */
    ub =  Ud * sin + Uq * cos;      /* β 轴电压 */

    /* ---- 2. 反 Clarke（等幅值）：三相调制波，A 相 = α 轴 ----
       标准：Va = uα, Vb = (−uα + √3·uβ)/2, Vc = (−uα − √3·uβ)/2；
       勿把 uα/uβ 对调（曾致电压向量整体偏 90°，半开环转矩方向错） */
    u1 = ua;
    u2 = (-ua + FOC_SQRT3 * ub) * 0.5f;
    u3 = (-ua - FOC_SQRT3 * ub) * 0.5f;

    /* ---- 3. 零序注入（min-max 中点偏移） ---- */
    umax = u1;
    if (u2 > umax) umax = u2;
    if (u3 > umax) umax = u3;
    umin = u1;
    if (u2 < umin) umin = u2;
    if (u3 < umin) umin = u3;
    offset = -(umax + umin) * 0.5f;

    *u = u1 + offset;
    *v = u2 + offset;
    *w = u3 + offset;

    /* ---- 4. 限幅到线性调制区（过调制削顶，避免桥臂越界） ---- */
    if (*u >  1.0f) *u =  1.0f;
    else if (*u < -1.0f) *u = -1.0f;
    if (*v >  1.0f) *v =  1.0f;
    else if (*v < -1.0f) *v = -1.0f;
    if (*w >  1.0f) *w =  1.0f;
    else if (*w < -1.0f) *w = -1.0f;

    /* ---- 5. 换算为桥臂占空比 [0, 1]：0 = 下桥臂全通、1 = 上桥臂全通、0.5 = 中点，
       PWM 层直接使用 ---- */
    *u = 0.5f + 0.5f * (*u);
    *v = 0.5f + 0.5f * (*v);
    *w = 0.5f + 0.5f * (*w);
}

/**
 * @brief  Clarke 变换（等幅值）：三相电流 → αβ 轴电流
 * @param  ia, ib, ic  三相电流（A）
 * @param  i_alpha     α 轴电流（= ia）
 * @param  i_beta      β 轴电流 = ia×(1/√3) + ib×(2/√3)
 * @note   三相平衡假设（星形绕组，无中线），ic 参数仅作占位；
 *         β 轴拆成两项常量系数乘法（无除法，M4F 可 FMA 融合为 2 条指令）：
 *         iβ = (ia + 2·ib)/√3 ≡ ia·(1/√3) + ib·(2/√3)
 */
void MC_Clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    (void)ic;   /* 三相平衡：ic = -(ia+ib)，β 轴只依赖 ia/ib */
    *i_alpha = ia;
    *i_beta  = ia * FOC_INV_SQRT3 + ib * FOC_2_INV_SQRT3;
}

/**
 * @brief  Park 变换：αβ 轴电流 → dq 轴电流（旋转坐标系）
 * @param  i_alpha  α 轴电流
 * @param  i_beta   β 轴电流
 * @param  sin, cos 电角度 sin(θe)/cos(θe)（调用方查表传入）
 * @param  i_d      d 轴电流（励磁分量）
 * @param  i_q      q 轴电流（转矩分量）
 */
void MC_Park(float i_alpha, float i_beta, float sin, float cos,
             float *i_d, float *i_q)
{
    *i_d =  cos * i_alpha + sin * i_beta;
    *i_q = -sin * i_alpha + cos * i_beta;
}
