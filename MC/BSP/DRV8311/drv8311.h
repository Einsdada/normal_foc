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

/* ============ 电流采样配置（按实际硬件/DRV8311 寄存器配置修改） ============
 * 单位链（最终必须是 A）：
 *   raw[LSB] → (raw − bias)[LSB] × A_PER_LSB[A/LSB] = I[A]
 *   A_PER_LSB = ADC_VREF[V] / ADC_RES[LSB] / GAIN_VA[V/A]
 *             = 3.3 / 4096 / 0.5 = 1.611e-3 A/LSB（≈1.61mA/LSB，满量程 ±2048LSB = ±3.3A）
 *   量纲核对：V/LSB ÷ V/A = A/LSB ✓
 *
 * 电机参数：额定 0.5A / 峰值 2A。
 * SOx 摆幅 ±1.65V（VREF=3.3V），各增益档满量程：
 *   2 V/A → ±0.825A（不够，2A 会饱和削顶）
 *   1 V/A → ±1.65A（不够）
 *   0.5V/A → ±3.3A（够，且留有余量）→ 选用本档
 *
 * @warning 下面 0.5 V/A **不是固件能配置/校验的**，它由硬件决定，必须对原理图确认：
 *   ① GAIN_VA[V/A] = CSA 增益[V/V] × 采样电阻[Ω]
 *      —— 本工程没有任何 DRV8311 寄存器配置代码（SPI3 只接编码器），
 *      所以增益只能靠 GAIN 引脚电阻（硬件变体 47kΩ±5% 接 GND）或 SPI 变体的
 *      上电默认值。若实际档位不是 0.5V/A，ch4~ch8 的电流会整体等比例偏大/偏小
 *      （1 V/A → 读数偏大 2 倍；2 V/A → 偏大 4 倍），角度/转速不受影响。
 *   ② ADC_VREF：本工程未开 VREFBUF，VREF+ 取 VDDA（按 3.3V）；若板子用外部基准
 *      （如 2.5V/3.0V）或 VDDA 不是 3.3V，必须同步改这里。
 *   ③ SOx 零点 = VREF/2：零点已由电流校准（bias）实测消除，只影响默认值，
 *      零点误差不影响换算精度；上电零电流校准时长 4096 点 ≈ 200ms。
 *
 * @note 符号约定：I = (raw − bias)×k 认为 raw > bias 为正电流。DRV8311 低边采样
 *       的正方向若与电机相序相反，则反馈电流符号整体翻转——电流环会变成正反馈
 *       （必须把 k 取负或调换 SOx 极性）。自检：半开环堵转下 ud=0、uq>0 时，
 *       按正确的电角度 park 出来应当是 Iq>0、Id≈0；若稳定读到 Iq<0，即符号反了。 */
#define DRV8311_GAIN_VA     0.5f   /* 电流采样总增益（V/A）：DRV8311 CSA 增益档位 */
#define DRV8311_SOx_VREF     1.65f  /* SOx 零点电压（V，静态零电流时输出 = VREF/2 = 3.3/2） */
#define DRV8311_ADC_VREF     3.3f   /* ADC 参考电压（V，VREF+ = VDDA，未开 VREFBUF） */
#define DRV8311_ADC_RES   4096.0f   /* ADC 12bit 满量程（LSB） */

/* 换算系数（由上面推导，编译期常量直接折叠）：I = (raw − bias) × A_PER_LSB
   供设备层 current_sensor 初始化转换系数 k 与默认偏置 */
/* 电流符号约定（调试开关）：
   +1.0f = raw > bias 视为正电流（默认）；-1.0f = 整条电流支路取反。
   什么时候改：电流环一闭环就正反馈跑飞（表现为立刻 `FAULT: OC trip`），
   说明低边采样极性与控制约定相反 → 改 -1.0f 重试。
   自检（改之前先做，最稳）：半开环堵转下 ud=0、uq>0 时，按已校准电角度 Park
   出来应当是 Iq>0、Id≈0；若稳定读到 Iq<0，就是这里要改 -1.0f */
#define DRV8311_I_SIGN      (+1.0f)

#define DRV8311_ZERO_CODE   ((uint16_t)(DRV8311_SOx_VREF * DRV8311_ADC_RES / DRV8311_ADC_VREF))  /* 理论零点码（= 2048，未校准时默认偏置） */
#define DRV8311_A_PER_LSB   (DRV8311_ADC_VREF / (DRV8311_ADC_RES * DRV8311_GAIN_VA))             /* 每 LSB 电流幅值（A/LSB，不含符号） */

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
/* 三相归一化占空比 0.0~1.0（A/B/C）——⚠ 调用方必须自己保证在 [0,1]：
   本函数为省掉 20kHz 中断里的分支**不再限幅**（SVPWM 与设备层已各钳一层） */
void  Drv8311_Pwm_SetDuty(float a, float b, float c);
void  Drv8311_Pwm_Enable(void);     /* 使能驱动器输出（sleep 拉高） */
void  Drv8311_Pwm_Disable(void);    /* 关断驱动器输出（sleep 拉低，三相关断） */

/* ---- 驱动器故障引脚 nFAULT（PC6，开漏，低有效） ----
   Init 里会把该脚配成带内部上拉的输入（板上若已有外部上拉则并联，无害）；
   返回 1 = 有故障（引脚低）。用途：排查"给了电压却不走电流"——
   DRV8311 处于 fault/欠压/过流时输出级直接关闭，固件原先完全看不到 */
uint8_t Drv8311_FaultRead(void);

#endif
