#ifndef __CURRENT_SENSOR_H__
#define __CURRENT_SENSOR_H__

#include "stdint.h"

/* =====================================================================
 * 三相电流传感设备层
 * - 原始码来自 BSP（drv8311_cur：ADC1 注入三通道，TIM1_CC4 触发 PWM 中点采样）
 * - 原始码 → 电流采用最小运算原则，先整型减法、再乘一次系数：
 *       I = (raw − bias) × k        bias 为整型（int32_t），raw 为整型原始码
 *       bias  每相零点偏置（校准中点算法得出，默认用硬件理论零点码）
 *       k     转换系数 A/LSB（编译期由硬件参数折叠，Init 时写入句柄）
 * - 校准算法即校准偏置值：电机零电流时采集 samples 次原始码取平均（四舍五入取整）
 * 实现：current_sensor.c
 * ===================================================================== */

/**
 * @brief  电流传感器类型
 */
typedef enum {
    CURRENT_SENSOR_3PH = 0,     // 三相采样：A/B/C 三相分别读码换算
    CURRENT_SENSOR_2PH = 1,     // 两相采样：只换算 A/B 两相，C 相由基尔霍夫定律计算（Ic = −(Ia+Ib)）
    CURRENT_SENSOR_RESERVED = 2,    // 预留
} CurrentSensor_Type_e;

/**
 * @brief  电流传感句柄
 */
typedef struct {
    CurrentSensor_Type_e type;    // 传感器类型
    float k;                      // 转换系数（A/LSB）：换算 1 次乘法
    int32_t bias[3];              // 每相零点偏置码（整型，校准得出）：换算 1 次整型减法
    float cur[3];                 // 每相电流缓存（A，Update 后有效）

    /* ---- 中点校准（中断内完成，单函数合成式）----
       校准 = 采固定 1024 次（2 的次方）零电流原始码取平均作为 bias，
       平均用位移除法 sum>>10；中断回调里每次转换完成调用一次，
       满 1024 后写 bias 并置 calib_done=1（据此切回 Update 分支） */
    volatile uint8_t  calib_done;    // 1 校准已完成（中断回调据此切换 Update 分支）
    uint32_t          calib_count;   // 已累加采样次数
    uint32_t          calib_sum[3];  // 三相累加和
    uint16_t          calib_target;  // 目标采样次数（宏固定 1024，登记时写入）
} CurrentSensor_Handle_t;

/**
 * @brief  初始化电流传感器
 * @param  handle  句柄
 * @param  type    传感器类型
 * @note   绑定 BSP 并启动注入转换（ADC 校准 + TIM1_CC4 触发 + 中断）；
 *         偏置默认取硬件理论零点码，上电零电流时调用 CurrentSensor_Calib 校准
 */
void CurrentSensor_Init(CurrentSensor_Handle_t *handle, CurrentSensor_Type_e type);

/**
 * @brief  更新三相电流：从 BSP 读原始码 → 换算（I = (raw − bias) × k）→ 缓存
 * @param  handle  句柄
 * @note   在注入转换完成中断里调用（40kHz 采样即得），或主循环按需调用
 */
void CurrentSensor_Update(CurrentSensor_Handle_t *handle);

#define CURRENT_SENSOR_CALIB_SAMPLES  4096u   /* 校准采样点数：固定 2 的次方（平均用位移除法） */
#define CURRENT_SENSOR_CALIB_SHIFT    12u     /* log2(4096)：平均 = sum >> 12 */

/**
 * @brief  校准偏置（中点算法）：零电流时采固定 1024 次三相原始码取平均 → bias[3]
 * @param  handle   句柄
 * @retval 1 校准已完成（或正在进行/兜底返回）
 * @note   纯累加版：登记已由 CurrentSensor_Init 完成（目标 1024、清零累加器）；
 *         注入转换完成中断回调按 calib_done 分流逐次调用本函数：
 *         未完成 → 本函数（按 type 分两套累加，满 1024 以位移除法 sum>>10 写 bias、
 *         置 calib_done=1），完成 → CurrentSensor_Update；
 *         必须在电机零电流（PWM 0% 输出、未使能驱动器）期间完成累加，运行中勿用
 */
uint8_t CurrentSensor_Calib(CurrentSensor_Handle_t *handle);

/**
 * @brief  获取 A 相电流（A）
 */
float CurrentSensor_GetIa(CurrentSensor_Handle_t *handle);

/**
 * @brief  获取 B 相电流（A）
 */
float CurrentSensor_GetIb(CurrentSensor_Handle_t *handle);

/**
 * @brief  获取 C 相电流（A）
 */
float CurrentSensor_GetIc(CurrentSensor_Handle_t *handle);

#endif
