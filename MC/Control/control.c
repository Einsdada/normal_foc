/**
 * @file    control.c
 * @brief   控制层实现：分频调度 + 三个频率环路 + 各模式处理
 *
 * 层级：User → Control → Device/BSP。
 *
 * 文件结构（按需跳转）：
 *   1. 实现常量        —— 只有本文件用的数学常量（可调参数全在 control.h）
 *   2. 内部工具        —— Foc_ObserveDq / Foc_GetSinCos / EncCalib_Reset
 *   3. 各模式处理      —— 一个模式一个函数，高频环每拍调用一个：
 *                          Mode_Stop / Mode_CurrentCalib / Mode_EncoderCalib /
 *                          Mode_FullOpenLoop / Mode_HalfOpenLoop / Mode_CurrentLoop
 *   4. 对外接口        —— Control_Init / Control_Sched / Control_HighLoop
 *                          / Control_MidLoop / Control_LowLoop
 *
 * 约定：
 *  - 所有状态统一封装在 MC_Handle_t（模式/目标/当前值/设备句柄/调度状态），
 *    函数统一注入 MC_Handle_t *handle，只操作 handle 指向的共享数据；
 *  - 每个环路先做【传感器信息更新】（switch 外、任何模式都运行，保证外部始终可读），
 *    再按 handle->mode 分发：
 *      Control_HighLoop（20kHz 每拍）  电流更新 + 模式处理
 *      Control_MidLoop （10kHz 2 分频）位置/角度/速度更新
 *      Control_LowLoop （5kHz  4 分频）位置环（待实现）
 *  - 单位：电流 A；角度 rad；Ud/Uq 与三相占空比为调制比（1.0 = Udc/2，非 V）。
 */
#include <stddef.h>     /* NULL */
#include "control.h"
#include "foc.h"         /* MC_SVPWM_Calc / MC_Clarke / MC_Park */
#include "mc_math.h"     /* MC_Math_Sin / MC_Math_Cos（查表法正余弦） */

/* ==================== 1. 实现常量 ====================
 * 可调参数（锁轴电压/稳定判据/方向判据/PI 初值）统一放在 control.h，此处只留
 * 本文件内部用的数学常量（不对外暴露通用名 FOC_2PI/FOC_PI，避免与 foc.h/mc_math.h 撞名） */
#define FOC_2PI    6.283185307179586f   /* 2π：开环电角度积分步长换算 */
#define FOC_PI     3.141592653589793f   /* π：角度解绕判据 */
#define FOC_HALF_PI (FOC_PI * 0.5f)     /* π/2：两点锁轴第二点的电角度（定方向用） */

/* ==================== 2. 内部工具 ==================== */

/**
 * @brief  三相电流 → 编码器电角度坐标系下的 Id/Iq（仅观测，不参与闭环）
 * @param  handle  数据库句柄
 * @param  s, c    编码器电角度的 sin/cos（调用方已算好，避免重复查表）
 * @note   结果写入 handle->state.id / state.iq，上位机看 ch7/ch8 即此值
 */
static void Foc_ObserveDq(MC_Handle_t *handle, float s, float c)
{
    float ialpha, ibeta;

    MC_Clarke(CurrentSensor_GetIa(&handle->device.cur),
              CurrentSensor_GetIb(&handle->device.cur),
              CurrentSensor_GetIc(&handle->device.cur),
              &ialpha, &ibeta);
    MC_Park(ialpha, ibeta, s, c, &handle->state.id, &handle->state.iq);
}

/**
 * @brief  取电流环/Park 用的电角度 sin/cos（含**角度延迟前推补偿**）
 * @param  handle  数据库句柄
 * @param  we      电角速度（rad/s，正 = 电角度增大方向；直接取 PosSensor_GetElecSpeed()）
 * @param  s, c    输出：补偿后的 sin/cos
 * @note   θ_used = θ_meas + ωe·ENC_ANGLE_LAG_US：编码器角度是"过去某一时刻"的
 *         （SSI 上一帧 + 中频环 10kHz 刷新 + PWM 生效延迟 ≈ 100~200us），转速越高
 *         滞后越大。不补偿时矢量在真实转子上被掰偏 ωe·Δt，转矩少 cos(...)、还要
 *         多输出 d 轴电压（实测顶速 Ud=−0.24 就是它）。推导与标定方法见 control.h
 */
static void Foc_GetSinCos(MC_Handle_t *handle, float we, float *s, float *c)
{
    float ang = PosSensor_GetElecAngle(&handle->device.pos)
              + we * (ENC_ANGLE_LAG_US * 1.0e-6f);

    MC_Math_SinCos(ang, s, c);      /* 一次算 sin/cos（省一次角度归一化，原 60 条 → ~35 条） */
}

/**
 * @brief  编码器校准状态机复位：回到第一个锁轴点（电角度 0°）
 * @note   进入校准模式、收尾阶段都调用；同时清掉方向判据的中间量
 */
static void EncCalib_Reset(MC_Handle_t *handle)
{
    handle->enc_calib.phase   = ENC_CALIB_PHASE_LOCK;
    handle->enc_calib.tick    = 0u;
    handle->enc_calib.n       = 0u;
    handle->enc_calib.sum     = 0.0f;
    handle->enc_calib.point   = 0u;
    handle->enc_calib.suspect = 0u;
    handle->enc_calib.mech0   = 0.0f;
    handle->enc_calib.dmech   = 0.0f;
}

/* ==================== 3. 各模式处理（高频环 20kHz 每拍） ==================== */

/**
 * @brief  停止：关断输出
 * @note   幂等，每拍都执行——保证从任意模式切到停止必定关断
 */
static void Mode_Stop(MC_Handle_t *handle)
{
    MotorPwm_Stop(&handle->device.pwm);     /* 占空比清零 + 关断驱动器（设备层安全态） */
}

/**
 * @brief  电流偏置校准：0% 占空比 + 使能驱动器，零电流累加求中点
 * @note   PWM 0%（下桥臂全通 → 绕组短路 → 零电流）；必须使能驱动器，因为
 *         DRV8311 关断（sleep 低）时 SOx 电流检测输出无有效零点，bias 会采错；
 *         再次校准只需重新进入本模式（CurrentSensor_Calib 内部自动重置重新开始）；
 *         校准完成后：已有编码器偏移/方向 → 直接回停止；否则自动进入编码器校准
 */
static void Mode_CurrentCalib(MC_Handle_t *handle)
{
    MotorPwm_SetDuty(&handle->device.pwm, 0.0f, 0.0f, 0.0f);   /* 0% 占空比 = 三相下桥臂全通 */
    MotorPwm_EnsureEnabled(&handle->device.pwm);                /* 使能驱动器：SOx 零点有效 */
    CurrentSensor_Calib(&handle->device.cur);

    if (handle->device.cur.calib_done)
    {
        EncCalib_Reset(handle);     /* 编码器校准状态机复位到第一个锁轴点 */
        handle->mode = handle->enc_calib_done ? MODE_STOP : MODE_ENCODER_CALIB;
    }
}

/**
 * @brief  编码器校准（两点锁轴法，阶段状态机）：同时定"零点"和"方向"
 *
 *   点 0：矢量锁在电角度 0°（Uα=U，即 A 相磁轴）→ 转子 d 轴对齐 α 轴
 *         → 等振动收敛 → 平均机械角 θmech0（50ms）
 *   点 1：矢量锁在电角度 +90°（Uβ=U）  → 转子 d 轴对齐 β 轴（再走 90° 电角度，
 *         机械只走 1/(4×极对数) 圈 ≈ 12.9°，不会产生 ±180° 包绕歧义）
 *         → 平均机械角 θmech1
 *
 *   两点联立（θe = dir×极对数×θmech + offset）：
 *     0   = dir×pp×θmech0 + offset
 *     π/2 = dir×pp×θmech1 + offset
 *   ⇒ dir    = sign(θmech1 − θmech0)   （机械角随电角度增大 = +1，反向 = −1）
 *     offset = NormRad(−dir×pp×θmech0)
 *
 * @note   单点锁轴只能定零点，"转起来"靠的是 dθe/dθmech 的**方向**：方向反时
 *         角度闭环（半开环/电流环）变正反馈——矢量相对转子倒转、转矩角立刻反向
 *         → 平均转矩 0 → 电机原地不转（只抖不动、电流全在 d 轴上），且与电压大小无关。
 *         故必须用第二点把方向测出来（方向参数在 control.h）。
 *         FINISH 关断回停止（enc_calib_done=1 + 置一次性打印标志）。
 */
static void Mode_EncoderCalib(MC_Handle_t *handle)
{
    float u, v, w;

    MotorPwm_EnsureEnabled(&handle->device.pwm);    /* 从停止切入：一次性使能 */

    switch (handle->enc_calib.phase)
    {
    case ENC_CALIB_PHASE_LOCK:      /* ---- 锁轴：固定矢量锁转子 + 等稳定 ---- */
        if (handle->enc_calib.point == 0u) {
            MC_SVPWM_Calc(ENC_CALIB_LOCK_VOLT, 0.0f, 0.0f, 1.0f, &u, &v, &w);  /* 点 0：Uα=U, Uβ=0（电角度 0°） */
        } else {
            MC_SVPWM_Calc(0.0f, ENC_CALIB_LOCK_VOLT, 0.0f, 1.0f, &u, &v, &w);  /* 点 1：Uα=0, Uβ=U（电角度 +90°） */
        }
        MotorPwm_SetDuty(&handle->device.pwm, u, v, w);

        {
            float mech = PosSensor_GetMechAngle(&handle->device.pos);   /* 原始机械角（校准专用） */
            if (handle->enc_calib.tick == 0u)
            {
                handle->enc_calib.first = mech;   /* min 初值 */
                handle->enc_calib.last  = mech;   /* max 初值 */
            }
            else
            {
                if (mech < handle->enc_calib.first) handle->enc_calib.first = mech;
                if (mech > handle->enc_calib.last)  handle->enc_calib.last  = mech;
            }
            handle->enc_calib.tick++;
        }

        /* 稳定判据：最小等待已过 && 位置波动 < 0.1° 机械；超时强制进读数
           @note 该判据只看"位置是否不再动"，编码器失效/转子被卡住时同样成立
                 （此时角度无声失效）——故点 1 之后还要用位移符号判方向，
                 位移不足则置 suspect 交主循环打印，避免"校准成功但角度是坏的" */
        if ((handle->enc_calib.tick >= ENC_CALIB_MIN_SETTLE &&
             (handle->enc_calib.last - handle->enc_calib.first) < ENC_CALIB_STABLE_ERR) ||
            handle->enc_calib.tick >= ENC_CALIB_MAX_SETTLE)
        {
            handle->enc_calib.phase = ENC_CALIB_PHASE_READ;
            handle->enc_calib.n     = 0u;
            handle->enc_calib.sum   = 0.0f;
        }
        break;

    case ENC_CALIB_PHASE_READ:      /* ---- 读数：多拍平均机械角 → 点 0 定零点 / 点 1 定方向 ---- */
        /* 继续锁轴，累加机械角 */
        handle->enc_calib.sum += PosSensor_GetMechAngle(&handle->device.pos);
        if (++handle->enc_calib.n >= ENC_CALIB_READ_N)
        {
            float mech_avg = handle->enc_calib.sum / (float)ENC_CALIB_READ_N;

            if (handle->enc_calib.point == 0u)
            {
                /* 点 0（真实电角度 0°）：记下 θmech0，转子再锁到电角度 +90° 一点 */
                handle->enc_calib.mech0 = mech_avg;
                handle->enc_calib.point = 1u;
                handle->enc_calib.phase = ENC_CALIB_PHASE_LOCK;
                handle->enc_calib.tick  = 0u;
                handle->enc_calib.n     = 0u;
                handle->enc_calib.sum   = 0.0f;
            }
            else
            {
                /* 点 1（真实电角度 +90°）：由两点位移定方向，再由点 0 定偏移 */
                float  dm     = mech_avg - handle->enc_calib.mech0;     /* 名义 ±π/(2×pp) */
                float  expect = FOC_HALF_PI / (float)MOTOR_POLE_PAIRS;  /* 理论机械位移 */
                float  admech;
                int8_t dir;
                float  off;

                /* 两个机械角各自在 [0, 2π)，差值可能跨过 0/2π 边界 → 解绕到 (−π, π] */
                if (dm > FOC_PI)       dm -= FOC_2PI;
                else if (dm <= -FOC_PI) dm += FOC_2PI;

                admech = (dm < 0.0f) ? -dm : dm;
                handle->enc_calib.dmech = dm;

                if (ENC_CALIB_DIR_FORCE != 0)
                {
                    /* 调试：手动强制方向，跳过自动判据（仍照常打印 dmech 供对照） */
                    handle->enc_calib.suspect = 0u;
                    dir = (ENC_CALIB_DIR_FORCE < 0) ? (int8_t)-1 : (int8_t)1;
                }
                else if (admech < (ENC_CALIB_DIR_MIN_RATIO * expect) ||
                         admech > (ENC_CALIB_DIR_MAX_RATIO * expect))
                {
                    /* 转子没走到第二点（被齿槽/摩擦卡住）或跳到远处平衡点：
                       位移符号不足以判方向 → 按常规方向 +1 兜底并置 suspect，
                       主循环把结果打印出来供人工确认（多为锁轴电压不足/机械卡死） */
                    handle->enc_calib.suspect = 1u;
                    dir = 1;
                }
                else
                {
                    handle->enc_calib.suspect = 0u;
                    dir = (dm >= 0.0f) ? (int8_t)1 : (int8_t)-1;
                }

                /* θmech0 处真实电角度 = 0 → offset = −dir×pp×θmech0 */
                off = -(float)dir * (float)MOTOR_POLE_PAIRS * handle->enc_calib.mech0;
                off -= (float)(int)(off * (1.0f / FOC_2PI)) * FOC_2PI;   /* 归 [0, 2π) */
                if (off < 0.0f) {
                    off += FOC_2PI;
                }

                PosSensor_SetElecDir(&handle->device.pos, dir);       /* 方向必须先于偏移生效 */
                PosSensor_SetElecOffset(&handle->device.pos, off);

                handle->enc_calib_done = 1u;
                handle->calib_report   = 1u;                          /* 主循环一次性打印校准结果 */
                handle->enc_calib.phase = ENC_CALIB_PHASE_FINISH;
            }
        }
        break;

    case ENC_CALIB_PHASE_FINISH:    /* ---- 收尾：关断回停止 ---- */
        /* 校准结果（偏移 + 方向）只保存在 RAM 里，每次上电重新校准：
           掉电保持（Flash 存取）属于后续开发内容，此处不再触发落盘 */
        MotorPwm_Stop(&handle->device.pwm);     /* 关断回停止 */
        /* 只复位过程量，便于再次校准；mech0/dmech/suspect 保留给主循环的一次性校准打印
           （下次校准点 0/点 1 会重新写入，不会串上一次的结果） */
        handle->enc_calib.phase = ENC_CALIB_PHASE_LOCK;
        handle->enc_calib.tick  = 0u;
        handle->enc_calib.n     = 0u;
        handle->enc_calib.sum   = 0.0f;
        handle->enc_calib.point = 0u;
        handle->mode = MODE_STOP;
        break;

    default:
        break;
    }
}

/**
 * @brief  全开环（VVVF）：无任何反馈，按给定电压幅值/电频率驱动电机旋转
 * @note   开环电角度每拍积分（θ += 2π·f / 20kHz）→ FOC SVPWM 输出；角度每拍归约回
 *         [0, 2π) 保证查表法正余弦的 float 精度；
 *         电压幅值 target.ol_volt（调制比 0~1，1 = Udc/2），电频率 target.ol_freq (Hz)；
 *         SVPWM 相电压为马鞍波（正弦+零序分量），线电压才是正弦，属正常现象；
 *         观察波形时电频率 ≤10Hz（串口 10ms 采样），否则欠采样看不到形状
 */
static void Mode_FullOpenLoop(MC_Handle_t *handle)
{
    float u, v, w;

    handle->state.ol_angle += FOC_2PI * handle->target.ol_freq / (float)CONTROL_HIGH_HZ;
    if (handle->state.ol_angle >= FOC_2PI) {
        handle->state.ol_angle -= FOC_2PI;
    }

    MotorPwm_EnsureEnabled(&handle->device.pwm);    /* 从停止切入：一次性使能（缓存占空比生效） */

    /* 编码器校准验证观测（仅观测，不参与驱动）：
       用校准后的编码器电角度做 Clarke/Park → Id/Iq。电机同步旋转时转子磁极位置
       ≈ 开环角度；若校准正确（offset 对、方向对），Id/Iq 为平稳直线（≈直流）；
       若偏移错/方向反，Id/Iq 呈正弦波动——上位机看 ch7/ch8 即可判断校准质量 */
    {
        float s_enc, c_enc;

        Foc_GetSinCos(handle, PosSensor_GetElecSpeed(&handle->device.pos), &s_enc, &c_enc);
        Foc_ObserveDq(handle, s_enc, c_enc);
    }

    /* 开环角度的 sin/cos：同样用合并查表（省一次角度归一化） */
    {
        float s_ol, c_ol;

        MC_Math_SinCos(handle->state.ol_angle, &s_ol, &c_ol);
        MC_SVPWM_Calc(0.0f, handle->target.ol_volt, s_ol, c_ol,
                      &u, &v, &w);            /* FOC SVPWM：输出三相占空比 [0, 1]，直接写入 */
    }
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/**
 * @brief  半开环（角度闭环 + 电流开环）
 * @note   电角度取编码器反馈（含校准偏移与方向，中频环更新）→ dq 旋转坐标与转子同步；
 *         电压直接给定（无 PI）：Ud = target.ud_target（通常 0），Uq = target.uq_target
 *         （**调制比**，1 = Udc/2，非 V）；
 *         用途：验证编码器校准/电角度对齐是否正确——对齐正确则电机平稳转动、
 *         串口 Id≈0 且 Iq 平稳；对齐错误则电流波动/卡顿；
 *         电流开环：堵转时电流只受 R 限制（I=U/R），测试时电压给小值（≤0.3）
 */
static void Mode_HalfOpenLoop(MC_Handle_t *handle)
{
    float u, v, w, s, c;
    /* 电角速度（rad/s）：位置传感器已按 机械 rps × 2π × 极对数 算好，供角度延迟补偿用 */
    float we = PosSensor_GetElecSpeed(&handle->device.pos);

    Foc_GetSinCos(handle, we, &s, &c);      /* 含角度延迟前推补偿 */

    MotorPwm_EnsureEnabled(&handle->device.pwm);    /* 从停止切入：一次性使能 */

    Foc_ObserveDq(handle, s, c);        /* 仅观测 Id/Iq 反馈，不闭环 */

    handle->state.ud = handle->target.ud_target;
    handle->state.uq = handle->target.uq_target;

    /* 注意单位：ud_target / uq_target 是调制比（1 = Udc/2），不是 V——
       要和"安培"的 id_target/iq_target 分清（半开环只给电压、不给电流） */
    MC_SVPWM_Calc(handle->target.ud_target, handle->target.uq_target, s, c, &u, &v, &w);
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/**
 * @brief  电流环闭环（角度闭环 + 电流闭环）
 * @note   编码器电角度 → Clarke/Park → dq 电流环（mc_current：PI + 前馈解耦 +
 *         矢量限幅 + 抗饱和）→ Ud/Uq → 逆 Park + SVPWM；给定 iq_target（A）后
 *         持续产生转矩 → 电机旋转（转速随电压可用量自由上升）；
 *         启动前提：电流校准 + 编码器校准已完成（就绪判断在 user 测试块）；
 *         电压限幅 CURRENT_U_MAX（矢量幅值，调制比）内自动避免过调制；
 *         带软件过流保护：连续超 CURRENT_I_TRIP 即关断并锁定（见下）
 */
static void Mode_CurrentLoop(MC_Handle_t *handle)
{
    float u, v, w;
    float s, c;
    float ud, uq, i2;

    /* 电角速度（rad/s，电机坐标系：正 = 电角度增大方向）：位置传感器按低通机械转速
       （中频环 10kHz 解算，fc≈95Hz）× 2π × 极对数给出；前馈与角度延迟补偿都用它 */
    float we = PosSensor_GetElecSpeed(&handle->device.pos);

    Foc_GetSinCos(handle, we, &s, &c);      /* 含角度延迟前推补偿 */

    /* Clarke + Park（编码器电角度）→ state.id / state.iq 反馈 */
    Foc_ObserveDq(handle, s, c);

    /* 软件过流保护：电流矢量幅值连续超阈值 → 关断 + 锁定（测试入口不再自动切入）。
       额定 0.5A、阈值 1.2A、连续 3 拍（0.15ms）触发。为什么需要：电流反馈符号若接反，
       闭环变正反馈，电流会顶到电压极限堵转值（1.10×6V/3.5Ω ≈ 1.9A），
       硬件 OCP 只保驱动器、这里保电机。用平方比较避免开方（工程无 libm） */
    i2 = handle->state.id * handle->state.id + handle->state.iq * handle->state.iq;
    if (i2 > (CURRENT_I_TRIP * CURRENT_I_TRIP))
    {
        if (++handle->oc_cnt >= CURRENT_I_TRIP_N)
        {
            handle->oc_cnt       = 0u;
            handle->oc_latched   = 1u;
            handle->fault_report = 1u;
            MotorPwm_Stop(&handle->device.pwm);     /* 关断输出（并锁定：测试入口不再自动切入） */
            CurrentLoop_Reset(&handle->current);
            handle->mode = MODE_STOP;
            return;
        }
    }
    else
    {
        handle->oc_cnt = 0u;
    }

    /* dq 电流环（给定 id_target/iq_target，单位 A；输出 Ud/Uq，单位调制比） */
    CurrentLoop_Run(&handle->current, 1.0f / (float)CONTROL_HIGH_HZ,
                    handle->target.id_target, handle->target.iq_target,
                    handle->state.id, handle->state.iq, we,
                    &ud, &uq);
    handle->state.ud = ud;
    handle->state.uq = uq;

    /* 从停止切入：驱动器先前关断 → 清电流环积分（避免带上一次运行的残留）并一次性使能。
       MotorPwm_EnsureEnabled 返回 1 表示"本次刚使能"，正好对应这个一次性动作 */
    if (MotorPwm_EnsureEnabled(&handle->device.pwm) != 0u) {
        CurrentLoop_Reset(&handle->current);
    }

    MC_SVPWM_Calc(ud, uq, s, c, &u, &v, &w);
    MotorPwm_SetDuty(&handle->device.pwm, u, v, w);
}

/* ==================== 4. 对外接口 ==================== */

/**
 * @brief  初始化：设备初始化 + 校准状态复位 + PI 参数
 * @note   上电默认进入电流校准模式，之后自动进入编码器校准（或跳过）→ 停止
 */
void Control_Init(MC_Handle_t *handle)
{
    /* 设备初始化（写入 handle->device） */
    MotorPwm_Init(&handle->device.pwm);             /* 启动三路载波，0% 占空比 */
    PosSensor_Init(&handle->device.pos, POS_SENSOR_SSI, MOTOR_POLE_PAIRS);   /* 位置传感器（SSI） */
    CurrentSensor_Init(&handle->device.cur, CURRENT_SENSOR_3PH);             /* ADC 校准 + CC4 触发 + 注入转换 */

    handle->sched.mid_cnt = 0u;
    handle->sched.low_cnt = 0u;

    /* 编码器校准状态机：初始化为第一个锁轴点（进入校准模式即从锁轴开始） */
    EncCalib_Reset(handle);

    /* 编码器校准偏移 + 方向：当前每次上电都重新执行编码器校准，
       不读非易失存储恢复、不落盘（掉电保持 = 后续开发内容）。
       后续要恢复时，在此处读出 offset/dir 后：
           方向必须先于偏移恢复（PosSensor_SetElecDir → PosSensor_SetElecOffset），
           两者必须成对使用（同一次校准的产物），恢复成功再把 enc_calib_done 置 1 */
    handle->enc_calib_done = 0u;    /* 强制上电执行编码器校准 */
    handle->calib_report   = 0u;
    handle->oc_latched     = 0u;    /* 过流锁定：上电清零 */
    handle->fault_report   = 0u;
    handle->oc_cnt         = 0u;

    /* 电流环：增益由电气参数 + 目标带宽算（推导见 control.h / mc_current.h），
       量出真实 R/L 后改宏即可；运行中也能直接改 mc.current.kp/ki 现场试 */
    {
        float wc     = FOC_2PI * CURRENT_BW_HZ;          /* 目标带宽（rad/s） */
        float u_half = 0.5f * MOTOR_UDC_NOMINAL;         /* 调制比 1.0 对应的相电压峰值 (V) */
        float kp     = (MOTOR_LD * wc) / u_half;         /* 调制比/A */
        float ki     = (MOTOR_R_PHASE * wc) / u_half;    /* 调制比/(A·s) */

        CurrentLoop_Init(&handle->current, kp, ki, CURRENT_U_MAX,
                         1.0f / u_half);                 /* v2m = 2/Udc：伏特→调制比 */
        CurrentLoop_SetFeedforward(&handle->current, MOTOR_LD, MOTOR_LQ,
                                   MOTOR_FLUX, (uint8_t)CURRENT_FF_EN);
    }

    handle->mode = MODE_CURRENT_CALIB;
}

/**
 * @brief  分频调度器（在 20kHz ADC 注入中断中调用）
 */
void Control_Sched(MC_Handle_t *handle)
{
    /* 分频调度：高频每拍；中频每 CONTROL_MID_DIV 拍（2 分频 = 10kHz）；
       低频每 CONTROL_LOW_DIV 拍（4 分频 = 5kHz） */
    Control_HighLoop(handle);

    if (++handle->sched.mid_cnt >= CONTROL_MID_DIV) {
        handle->sched.mid_cnt = 0u;
        Control_MidLoop(handle);
    }
    if (++handle->sched.low_cnt >= CONTROL_LOW_DIV) {
        handle->sched.low_cnt = 0u;
        Control_LowLoop(handle);
    }
}

/**
 * @brief 高频环（20kHz，每拍）：传感器更新 + 按模式分发到各 Mode_xxx
 */
void Control_HighLoop(MC_Handle_t *handle)
{
    /* ---- 传感器信息更新（switch 外，任何模式都运行） ---- */
    CurrentSensor_Update(&handle->device.cur);      /* 一直更新：每拍换算三相电流 */

    /* ---- 模式分发：具体实现在本文件第 3 节 ---- */
    switch (handle->mode)
    {
    case MODE_STOP:                   Mode_Stop(handle);         break;
    case MODE_CURRENT_CALIB:          Mode_CurrentCalib(handle); break;
    case MODE_ENCODER_CALIB:          Mode_EncoderCalib(handle); break;
    case MODE_FULL_OPEN_LOOP:         Mode_FullOpenLoop(handle); break;
    case MODE_HALF_OPEN_LOOP:         Mode_HalfOpenLoop(handle); break;
    case MODE_CURRENT_LOOP:           Mode_CurrentLoop(handle);  break;
    case MODE_SPEED_CURRENT_LOOP:
    case MODE_POS_SPEED_CURRENT_LOOP: /* TODO: 速度环 / 位置环（输出 → Iq 给定） */ break;
    default:                          break;
    }
}

/**
 * @brief 中频环（10kHz，2 分频）
 * @note  位置/角度/速度更新在 switch 外（任何模式都运行）；
 *        再按模式分发（速度环待实现）
 */
void Control_MidLoop(MC_Handle_t *handle)
{
    /* ---- 传感器信息更新（switch 外，任何模式都运行） ---- */
    PosSensor_Update(&handle->device.pos, CONTROL_MID_HZ);  /* 位置/角度/速度 */

    /* ---- 模式分发 ---- */
    switch (handle->mode)
    {
    case MODE_SPEED_CURRENT_LOOP:
    case MODE_POS_SPEED_CURRENT_LOOP:
        /* TODO: 速度环（handle->target.spd_target / 位置环输出 → Iq 给定） */
        break;

    default:
        break;
    }
}

/**
 * @brief 低频环（5kHz，4 分频）
 * @note  按模式分发（位置环待实现）
 */
void Control_LowLoop(MC_Handle_t *handle)
{
    switch (handle->mode)
    {
    case MODE_POS_SPEED_CURRENT_LOOP:
        /* TODO: 位置环（handle->target.pos_target → 速度给定） */
        break;

    default:
        break;
    }
}
