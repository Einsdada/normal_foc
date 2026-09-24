#include "current_sensor.h"
#include "drv8311.h"        /* BSP 层：ADC 原始码读取 + 换算常数 */

/**
 * @brief  初始化电流传感器
 * @note   绑定 BSP 并启动注入转换（ADC 校准 + TIM1_CC4 触发 + 中断）；
 *         偏置默认取硬件理论零点码；校准状态初始化为未完成，
 *         由校准模式调用 CurrentSensor_Calib 逐次累加
 */
void CurrentSensor_Init(CurrentSensor_Handle_t *handle, CurrentSensor_Type_e type)
{
    handle->type = type;

    switch (type)
    {
    case CURRENT_SENSOR_3PH:
    case CURRENT_SENSOR_2PH:                    /* 两相模式硬件启动与三相相同（注入转换三通道都在跑） */
        handle->k = DRV8311_I_SIGN * DRV8311_A_PER_LSB;         /* 转换系数（A/LSB，含符号约定 +1/-1） */
        handle->bias[0] = handle->bias[1] = handle->bias[2] = (int32_t)DRV8311_ZERO_CODE;  /* 默认理论零点码 */
        handle->cur[0]  = handle->cur[1]  = handle->cur[2]  = 0.0f;

        /* 校准状态初始化为未完成（累加由校准模式调用 CurrentSensor_Calib 逐次进行） */
        handle->calib_target = CURRENT_SENSOR_CALIB_SAMPLES;
        handle->calib_sum[0] = handle->calib_sum[1] = handle->calib_sum[2] = 0u;
        handle->calib_count  = 0u;
        handle->calib_done   = 0u;

        Drv8311_Cur_Init();     /* 启动注入转换：ADC 校准 + TIM1_CC4 触发 + 中断 */
        break;
    default:
        break;                  /* 预留类型暂不初始化 */
    }
}

/**
 * @brief  更新三相电流：从 BSP 读原始码 → 换算（最小运算 1 减 1 乘）→ 缓存
 * @note   在注入转换完成中断里调用（20kHz 采样即得，中心对齐模式）
 */
void CurrentSensor_Update(CurrentSensor_Handle_t *handle)
{
    uint16_t raw_a, raw_b, raw_c;

    switch (handle->type)
    {
    case CURRENT_SENSOR_3PH:
        Drv8311_Cur_ReadRaw(&raw_a, &raw_b, &raw_c);   /* BSP 只给三相原始码，不做换算 */
        handle->cur[0] = (float)((int32_t)raw_a - handle->bias[0]) * handle->k;  /* 整型减法 → 1 次乘法 */
        handle->cur[1] = (float)((int32_t)raw_b - handle->bias[1]) * handle->k;
        handle->cur[2] = (float)((int32_t)raw_c - handle->bias[2]) * handle->k;
        break;

    case CURRENT_SENSOR_2PH:
        /* 两相采样：只换算 A/B 两相，C 相由基尔霍夫定律计算（Ic = −(Ia+Ib)），
           省去 C 相偏置与换算，且天然满足三相电流之和为零 */
        Drv8311_Cur_ReadRaw(&raw_a, &raw_b, &raw_c);
        handle->cur[0] = (float)((int32_t)raw_a - handle->bias[0]) * handle->k;
        handle->cur[1] = (float)((int32_t)raw_b - handle->bias[1]) * handle->k;
        handle->cur[2] = -(handle->cur[0] + handle->cur[1]);
        break;

    default:
        break;
    }
}

/**
 * @brief  校准偏置（中点算法）：零电流时采固定 4096 次三相原始码取平均 → bias[3]
 * @note   累加式：ADC 注入转换完成中断（20kHz）里逐次调用本函数，
 *         按 switch(type) 分两套累加，满 4096 后以位移除法写 bias、置 calib_done=1；
 *         已校准过（calib_done=1）时再次调用会自动清零累加器重新开始，
 *         因此再次校准只需重新运行本函数，无需单独重置；
 *         必须在电机零电流（PWM 0% 输出、未使能驱动器）期间完成累加——
 *         由校准模式保证（进入时关断输出）
 */
uint8_t CurrentSensor_Calib(CurrentSensor_Handle_t *handle)
{
    uint16_t raw_a, raw_b, raw_c;
    uint32_t half;

    if (handle->calib_done) {
        /* 已完成过：自动清零，重新开始一轮校准（再次校准直接调用本函数即可） */
        handle->calib_sum[0] = handle->calib_sum[1] = handle->calib_sum[2] = 0u;
        handle->calib_count  = 0u;
        handle->calib_done   = 0u;
    }

    Drv8311_Cur_ReadRaw(&raw_a, &raw_b, &raw_c);   /* 中断回调本身在转换完成后执行，JDR 必然有效 */
    half = (uint32_t)CURRENT_SENSOR_CALIB_SAMPLES >> 1u;   /* 四舍五入半数 = 2048 */

    /* 三相一套 / 两相一套，各自独立 */
    switch (handle->type)
    {
    case CURRENT_SENSOR_3PH:            /* 三相：A/B/C 全部累加、全部校准 */
        handle->calib_sum[0] += raw_a;
        handle->calib_sum[1] += raw_b;
        handle->calib_sum[2] += raw_c;
        if (++handle->calib_count >= handle->calib_target)
        {
            handle->bias[0] = (int32_t)((handle->calib_sum[0] + half) >> CURRENT_SENSOR_CALIB_SHIFT);
            handle->bias[1] = (int32_t)((handle->calib_sum[1] + half) >> CURRENT_SENSOR_CALIB_SHIFT);
            handle->bias[2] = (int32_t)((handle->calib_sum[2] + half) >> CURRENT_SENSOR_CALIB_SHIFT);
            handle->calib_done = 1u;    /* 中断回调切回 Update 分支 */
        }
        break;

    case CURRENT_SENSOR_2PH:            /* 两相：只校 A/B，C 相由基尔霍夫定律计算 */
        handle->calib_sum[0] += raw_a;
        handle->calib_sum[1] += raw_b;
        if (++handle->calib_count >= handle->calib_target)
        {
            handle->bias[0] = (int32_t)((handle->calib_sum[0] + half) >> CURRENT_SENSOR_CALIB_SHIFT);
            handle->bias[1] = (int32_t)((handle->calib_sum[1] + half) >> CURRENT_SENSOR_CALIB_SHIFT);
            handle->calib_done = 1u;
        }
        break;

    default:
        break;
    }
    return 1u;
}

/**
 * @brief  获取 A 相电流（A）
 */
float CurrentSensor_GetIa(CurrentSensor_Handle_t *handle)
{
    return handle->cur[0];
}

/**
 * @brief  获取 B 相电流（A）
 */
float CurrentSensor_GetIb(CurrentSensor_Handle_t *handle)
{
    return handle->cur[1];
}

/**
 * @brief  获取 C 相电流（A）
 */
float CurrentSensor_GetIc(CurrentSensor_Handle_t *handle)
{
    return handle->cur[2];
}
