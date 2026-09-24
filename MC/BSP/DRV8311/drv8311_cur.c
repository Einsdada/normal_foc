#include "drv8311.h"
#include "adc.h"
#include "tim.h"        /* htim1：CC4 触发源 */

extern ADC_HandleTypeDef hadc1;

/**
 * @brief  启动注入转换：ADC 校准 + TIM1_CC4 触发（PWM 中点） + 中断
 */
void Drv8311_Cur_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* 采样时刻：触发源是 TIM1_CC4，必须写 CCR4（不能写 CCR1！）
       中心对齐 + PWM1：高边在"波谷(CNT=0)"附近导通、低边在"波峰(CNT=ARR)"附近导通，
       所以**计数器波峰**才是"三相低边全通（零矢量）"区间的中心：
         - 占空比 ≤1 ⇒ CCR ≤ ARR ⇒ 波峰处 CNT<CCR 必不成立 ⇒ 三相高边必定全关，
           低边分流电阻一定在流过相电流，读数有效（与占空比大小无关）；
         - 离最近一次开关动作最远 → 电流传感器建立时间最长、开关噪声最小；
         - 允许调制比一直用到 SVPWM 线性区上限 1.1547。
       旧值 0.95*ARR 的问题：调制比 >~1.04 时最大占空比 >0.95，采样点落进该相
       高边"仍在导通"的区间 → 那一相电流读数被污染。这等于把最高转速卡在 ~2300rpm
       （12V 下电机规格 2600rpm 上不去的原因之一，见 control.h 的 CURRENT_U_MAX）*/
    TIM1->CCR4 = htim1.Instance->ARR - 1u;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
 * @brief  读三相原始 ADC 码（寄存器直读，非阻塞）
 * @param  raw_a / raw_b / raw_c   A / B / C 相原始码输出
 * @note   ADC1 配置为 12bit 右对齐，注入结果在 JDRx[15:0]；
 *         直读寄存器替代 HAL_ADCEx_InjectedGetValue（省去参数检查/对齐分支），
 *         供 20kHz 注入中断内快速取数
 *         原始码 → 电流的换算由设备层 current_sensor 完成：
 *         I = (raw − bias) × A_PER_LSB，bias 由中点校准得出
 */
void Drv8311_Cur_ReadRaw(uint16_t *raw_a, uint16_t *raw_b, uint16_t *raw_c)
{
    *raw_a = (uint16_t)(hadc1.Instance->JDR1);   /* ADC1 注入数据寄存器 1：RANK1 = CH12 = A 相 */
    *raw_b = (uint16_t)(hadc1.Instance->JDR2);   /* ADC1 注入数据寄存器 2：RANK2 = CH14 = B 相 */
    *raw_c = (uint16_t)(hadc1.Instance->JDR3);   /* ADC1 注入数据寄存器 3：RANK3 = CH11 = C 相 */
}
