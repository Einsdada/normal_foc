#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"

#include "encoder.h"
uint16_t User_GetAngle(void) {
    Encoder_RawData_t data;
    if(Encoder_Process_RawData(&data))// 校验通过
    {
        return data.angle;// 原始值，长度16384，0~16383
    }
    return 0;
}

/* ===================== 芯片中断回调函数 ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		Led_Loop();     /* 1ms 一次，推进呼吸灯 */
        uart_printf("Angle: %d\r\n", User_GetAngle());
	}
}

/* ===================== 初始化与循环 ===================== */
void User_Init(void)
{
    Debug_Usart_Init();                     /* 调试串口 */

    Led_Init();                             /* 呼吸灯 PWM */

    HAL_TIM_Base_Start_IT(&htim6);          /* 启动 1ms 定时中断 */

    Encoder_HW_Init();
}

void User_Loop(void)
{

    // HAL_Delay(100);
}
