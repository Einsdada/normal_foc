#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"

/* ===================== 芯片中断回调函数 ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		Led_Loop();     /* 1ms 一次，推进呼吸灯 */
	}
}

/* ===================== 初始化与循环 ===================== */
void User_Init(void)
{
    Debug_Usart_Init();                     /* 调试串口 */

    Led_Init();                             /* 呼吸灯 PWM */

    HAL_TIM_Base_Start_IT(&htim6);          /* 启动 1ms 定时中断 */
}

void User_Loop(void)
{
    uart_printf("Hello, World!\n");
    HAL_Delay(500);
}
