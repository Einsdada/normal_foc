#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"

#include "pos_sensor.h"

#define MOTOR_POLE_PAIRS   7        /* 电机极对数，按实际电机修改 */
#define SENSOR_FREQ_HZ    1000     /* 位置传感器更新频率（Hz）= TIM6 1ms */
#define PRINT_PERIOD_MS    10       /* 串口打印周期（ms），在主循环执行 */

static PosSensor_Handle_t g_pos_sensor;   /* 位置传感器设备句柄 */
static Led_Handle_t      g_led_G;         /* 绿色 LED 设备句柄 */
static volatile uint8_t  g_print_flag = 0;   /* 主循环打印标志 */
static uint16_t          g_print_cnt  = 0;   /* 打印周期计数 */

/* ===================== 芯片中断回调函数 ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Led_Update(&g_led_G, SENSOR_FREQ_HZ);       /* 1ms 中断调用 = 1000Hz，更新 LED 状态机 */

        PosSensor_Update(&g_pos_sensor, SENSOR_FREQ_HZ);   /* 1kHz 采样一次位置传感器 */

        if (++g_print_cnt >= PRINT_PERIOD_MS)   /* 每 10ms 置打印标志 */
        {
            g_print_cnt  = 0;
            g_print_flag = 1;
        }
    }
}

/* ===================== 初始化与循环 ===================== */
void User_Init(void)
{
    Debug_Usart_Init();             /* 调试串口 */

    Led_Init(&g_led_G, 0);                              /* LED0 绑定 BSP 通道 0（BSP 绑定在设备层内部完成） */
    Led_SetBreathe(&g_led_G, 255, 0, 2500, LED_WAVE_SINE);   /* 默认正弦呼吸（峰值 255 / 谷值 150） */

    PosSensor_Init(&g_pos_sensor, POS_SENSOR_SSI, MOTOR_POLE_PAIRS);   /* 跳线帽当前接 SSI 模式 */

    HAL_TIM_Base_Start_IT(&htim6);  /* 启动 1ms 定时中断 */
}

void User_Loop(void)
{
    if (g_print_flag)               /* 主循环里打印，避开中断（%f 较慢） */
    {
        g_print_flag = 0;
        /* FireWater：纯数值逗号分隔 + 换行结尾
           ch0=Mech(rad) ch1=Elec(rad) ch2=Rpm ch3=Pos(rad) */
        uart_printf("%.3f,%.3f,%.1f,%.3f\n",
                    PosSensor_GetMechAngle(&g_pos_sensor),
                    PosSensor_GetElecAngle(&g_pos_sensor),
                    PosSensor_GetSpeedRpm(&g_pos_sensor),
                    PosSensor_GetPosition(&g_pos_sensor));
    }
}
