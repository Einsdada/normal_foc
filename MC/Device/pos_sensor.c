#include "pos_sensor.h"
#include "mt6701.h"         /* BSP 层：MT6701 磁编码器 */

#define TWO_PI              6.2831853f  /* 圆周率 2π */
#define PI                  3.14159265f /* 圆周率 π */

/* 固定常量运算用括号包起来，编译期即折叠为单个常量，运行时无除法 */
#define INV_TWO_PI          (1.0f / TWO_PI)                     /* 1/2π ≈ 0.159155 */
#define SSI_RAD_PER_COUNT   (TWO_PI / (float)MT6701_SSI_CPR)    /* SSI 每计数弧度 */
#define ABZ_RAD_PER_COUNT   (TWO_PI / (float)MT6701_ABZ_CPR)    /* ABZ 每计数弧度 */

/**
 * @brief  角度取模到 [0, 2π)
 */
static float NormRad(float rad)
{
    rad -= (float)(int)(rad * INV_TWO_PI) * TWO_PI;   /* 取模用乘倒数，无除法 */
    if (rad < 0.0f)
        rad += TWO_PI;
    return rad;
}

/**
 * @brief  角度差（rad）：处理 ±π 过零，返回有符号角度差
 */
static float AngleDelta(float new_rad, float old_rad)
{
    float d = new_rad - old_rad;
    if (d > PI)
        d -= TWO_PI;
    else if (d <= -PI)
        d += TWO_PI;
    return d;
}

/**
 * @brief  初始化位置传感器
 */
void PosSensor_Init(PosSensor_Handle_t *handle, PosSensor_Type_e type, uint16_t pole_pairs)
{
    handle->type            = type;
    handle->pole_pairs      = pole_pairs;
    handle->mech_angle      = 0.0f;
    handle->mech_angle_last = 0.0f;
    handle->elec_angle      = 0.0f;
    handle->mech_speed      = 0.0f;
    handle->position        = 0.0f;
    handle->valid           = 0;

    switch (type)
    {
    case POS_SENSOR_SSI:
        MT6701_Ssi_Init();                  /* 启动 SSI 编码器硬件 */
        break;
    case POS_SENSOR_ABZ:
        MT6701_Abz_Init();                  /* 启动 ABZ 编码器计数 */
        break;
    default:
        break;                              /* 预留类型暂不初始化 */
    }
}

/**
 * @brief  更新一次角度/速度/位置数据，返回是否有有效数据
 */
uint8_t PosSensor_Update(PosSensor_Handle_t *handle, uint16_t freq)
{
    Encoder_RawData_t data;

    switch (handle->type)
    {
    case POS_SENSOR_SSI:
        /* SSI 解算：14bit 绝对角度 → 电角度 / 速度 / 绝对位置（单位 rad、rps） */
        if (MT6701_Ssi_Process_RawData(&data))      /* BSP 层读到有效帧 */
        {
            handle->mech_angle  = (float)data.angle * SSI_RAD_PER_COUNT;   /* 1 次乘法 */
            // handle->elec_angle  = NormRad(handle->mech_angle * (float)handle->pole_pairs);
            handle->elec_angle  = handle->mech_angle * (float)handle->pole_pairs;
            if (handle->valid)                      /* 已有上一帧才解算速度/位置 */
            {
                float delta = AngleDelta(handle->mech_angle, handle->mech_angle_last);
                handle->mech_speed = delta * INV_TWO_PI * (float)freq;   /* rps，无除法 */
                handle->position  += delta;                     /* rad */
            }
            handle->mech_angle_last = handle->mech_angle;
            handle->valid           = 1;
        }
        break;

    case POS_SENSOR_ABZ:
        /* ABZ 解算：增量计数取模归一化 → 电角度 / 速度 / 绝对位置（单位 rad、rps） */
        {
            int32_t m = MT6701_Abz_GetCount() % MT6701_ABZ_CPR;
            if (m < 0) m += MT6701_ABZ_CPR;
            handle->mech_angle  = (float)m * ABZ_RAD_PER_COUNT;             /* 1 次乘法 */
            // handle->elec_angle  = NormRad(handle->mech_angle * (float)handle->pole_pairs);
            handle->elec_angle  = handle->mech_angle * (float)handle->pole_pairs;
            if (handle->valid)                      /* 已有上一帧才解算速度/位置 */
            {
                float delta = AngleDelta(handle->mech_angle, handle->mech_angle_last);
                handle->mech_speed = delta * INV_TWO_PI * (float)freq;   /* rps，无除法 */
                handle->position  += delta;                     /* rad */
            }
            handle->mech_angle_last = handle->mech_angle;
            handle->valid           = 1;
        }
        break;

    default:
        handle->valid       = 0;                    /* 预留类型暂无数据 */
        break;
    }
    return handle->valid;
}

/**
 * @brief  获取机械角度（rad）
 */
float PosSensor_GetMechAngle(PosSensor_Handle_t *handle)
{
    return handle->mech_angle;
}

/**
 * @brief  获取电角度（rad）
 */
float PosSensor_GetElecAngle(PosSensor_Handle_t *handle)
{
    return handle->elec_angle;
}

/**
 * @brief  获取机械转速（rps，转/秒）
 */
float PosSensor_GetSpeed(PosSensor_Handle_t *handle)
{
    return handle->mech_speed;
}

/**
 * @brief  获取机械转速（rpm，转/分）= rps × 60
 */
float PosSensor_GetSpeedRpm(PosSensor_Handle_t *handle)
{
    return handle->mech_speed * 60.0f;
}

/**
 * @brief  获取绝对位置（rad）
 */
float PosSensor_GetPosition(PosSensor_Handle_t *handle)
{
    return handle->position;
}
