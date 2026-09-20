#ifndef __DRV8311_H__
#define __DRV8311_H__

#include "stdint.h"

/* =====================================================================
 * DRV8311 三相电机驱动器板级驱动（BSP 层）
 *
 * 电流采样（ADC1 注入三通道，TIM1_CC4 触发 PWM 中点同步采样）：
 *   RANK1 = ADC1_IN12 = PB1  = SOA  → A 相电流
 *   RANK2 = ADC1_IN14 = PB11 = SOB  → B 相电流
 *   RANK3 = ADC1_IN11 = PB12 = SOC  → C 相电流
 *
 * 本层只负责硬件启动与读取三通道原始 ADC 码，不做电流换算；
 * 原始码 → 电流的换算（I = (raw − bias) × A_PER_LSB，最小运算 1 减 1 乘）
 * 由设备层 current_sensor 完成，bias 为其中点校准得出的偏置值
 * 实现：drv8311_cur.c（电流采样）/ drv8311_pwm.c（PWM 输出）
 * ===================================================================== */

/* ============ 电流采样配置（按实际硬件/DRV8311 寄存器配置修改） ============ */
/* 电机参数：额定 0.5A / 峰值 2A。
   SOx 摆幅 ±1.65V（VREF=3.3V），各增益档满量程：
     2 V/A → ±0.825A（不够，2A 会饱和削顶）
     1 V/A → ±1.65A（不够）
     0.5V/A → ±3.3A（够，且留有余量）→ 选用本档
   硬件对应：CSA_GAIN=01（SPI 变体）或 GAIN 引脚 47kΩ±5% 接 GND（硬件变体） */
#define DRV8311_GAIN_VA     0.5f   /* 电流采样总增益（V/A）：DRV8311 CSA 增益档位 */
#define DRV8311_SOx_VREF     1.65f  /* SOx 零点电压（V，静态零电流时输出 = VREF/2 = 3.3/2） */
#define DRV8311_ADC_VREF     3.3f   /* ADC 参考电压（V） */
#define DRV8311_ADC_RES   4096.0f   /* ADC 12bit 满量程 */

/* 换算系数（由上面推导，编译期常量直接折叠）：I = (raw − bias) × A_PER_LSB
   供设备层 current_sensor 初始化转换系数 k 与默认偏置 */
#define DRV8311_ZERO_CODE   ((uint16_t)(DRV8311_SOx_VREF * DRV8311_ADC_RES / DRV8311_ADC_VREF))  /* 理论零点码（= 2048，未校准时默认偏置） */
#define DRV8311_A_PER_LSB   (DRV8311_ADC_VREF / (DRV8311_ADC_RES * DRV8311_GAIN_VA))             /* 每 LSB 电流（A） */

/* ============ PWM 输出配置（TIM1 三线 → DRV8311 三相输入） ============ */
#define DRV8311_PWM_ARR       4250   /* TIM1 ARR（中心对齐，170MHz/2/4250 ≈ 20kHz） */
                                     /* 占空比接口统一归一化：0.0~1.0，BSP 内部映射到 ARR */

/* ============ 对外接口（实现：drv8311_cur.c / drv8311_pwm.c） ============ */
/* ---- 电流采样原始值（换算由设备层 current_sensor 完成） ---- */
void     Drv8311_Cur_Init(void);                          /* 启动注入转换：ADC 校准 + TIM1_CC4 触发 + 中断 */
void     Drv8311_Cur_ReadRaw(uint16_t *raw_a,             /* 读三相原始 ADC 码（非阻塞，读注入数据寄存器） */
                             uint16_t *raw_b,
                             uint16_t *raw_c);

void  Drv8311_Pwm_Init(void);       /* 启动三路 PWM 输出（初始 0% 占空比） */
void  Drv8311_Pwm_SetDuty(float a, float b, float c);   /* 三相归一化占空比 0.0~1.0（A/B/C） */
void  Drv8311_Pwm_Enable(void);     /* 使能驱动器输出（sleep 拉高） */
void  Drv8311_Pwm_Disable(void);    /* 关断驱动器输出（sleep 拉低，三相关断） */

#endif
