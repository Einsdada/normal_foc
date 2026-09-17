#include "mt6701.h"
#include "tim.h"

/**
 * @brief  启动 TIM2 编码器计数
 */
void MT6701_Abz_Init(void)
{
    /* 上电先配置芯片 ABZ 分辨率（RAM 配置断电丢失，每次上电写一次）
       MT6701_ABZ_CPR 是四倍频计数，/4 = 线数 */
    MT6701_Abz_Configure((uint16_t)(MT6701_ABZ_CPR / 4U));

    /* MODE 拉低选择 ABZ 模式（SSI 模式为高） */
    MT6701_SSI_MODE_PORT->BSRR = (uint32_t)MT6701_SSI_MODE_PIN << 16U;

    MT6701_Abz_Clear();
    /* TIM2 已在 CubeMX（MX_TIM2_Init）配置为编码器模式，这里启动计数 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取当前计数值
 */
int32_t MT6701_Abz_GetCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief  计数值清零
 */
void MT6701_Abz_Clear(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}
