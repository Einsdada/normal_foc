#ifndef __POS_SENSOR_H__
#define __POS_SENSOR_H__

#include "stdint.h"

/**
 * @brief  位置传感器类型
 * @note   位置传感器抽象层：SSI/ABZ 已实现，霍尔预留
 */
typedef enum {
    POS_SENSOR_SSI      = 0,   // SSI 绝对编码器（BSP 层 mt6701_ssi）
    POS_SENSOR_ABZ      = 1,   // ABZ 增量编码器（BSP 层 mt6701_abz）
    POS_SENSOR_HALL     = 2,   // 霍尔传感器（预留）
} PosSensor_Type_e;

/**
 * @brief  位置传感器句柄
 * @note   单位统一：角度/位置 rad，转速 rps（转/秒），rpm 由接口换算
 */
typedef struct {
    PosSensor_Type_e type;         // 传感器类型
    uint16_t         pole_pairs;   // 电机极对数（机械角度换算电角度用）
    float            mech_angle;   // 机械角度（rad）
    float            mech_angle_last;  // 上次机械角度（解算速度/绝对位置用）
    float            elec_angle;   // 电角度（rad）
    float            mech_speed;   // 机械转速（rps，正=正转）
    float            position;     // 绝对位置（rad，含整圈，反转时为负）
    uint8_t          valid;        // 最新一帧是否有效
} PosSensor_Handle_t;

/**
 * @brief  初始化位置传感器
 * @param  handle       句柄
 * @param  type         传感器类型
 * @param  pole_pairs   电机极对数（≥1）
 * @note   每圈计数由各 BSP 层宏定义（MT6701_SSI_CPR / MT6701_ABZ_CPR）
 */
void PosSensor_Init(PosSensor_Handle_t *handle, PosSensor_Type_e type, uint16_t pole_pairs);

/**
 * @brief  更新一次角度/速度/位置数据
 * @param  handle  句柄
 * @param  freq    本函数调用频率（Hz，如 1ms 调用 = 1000），用于解算速度
 * @retval 1=读到有效数据，0=暂无有效数据
 * @note   需要连续读取时在循环里以固定周期反复调用
 */
uint8_t PosSensor_Update(PosSensor_Handle_t *handle, uint16_t freq);

/**
 * @brief  获取机械角度（rad）
 */
float PosSensor_GetMechAngle(PosSensor_Handle_t *handle);

/**
 * @brief  获取电角度（rad）
 * @note   已取模到 [0, 2π)
 */
float PosSensor_GetElecAngle(PosSensor_Handle_t *handle);

/**
 * @brief  获取机械转速（rps，转/秒）
 */
float PosSensor_GetSpeed(PosSensor_Handle_t *handle);

/**
 * @brief  获取机械转速（rpm，转/分）= rps × 60
 */
float PosSensor_GetSpeedRpm(PosSensor_Handle_t *handle);

/**
 * @brief  获取绝对位置（rad，含整圈，反转时为负）
 */
float PosSensor_GetPosition(PosSensor_Handle_t *handle);

#endif
