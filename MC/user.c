#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"

#include "pos_sensor.h"
#include "mt6701.h"        /* 临时诊断：ABZ 原始计数 */

#define MOTOR_POLE_PAIRS   7        /* 电机极对数，按实际电机修改 */
#define SENSOR_TICK_S      0.001f   /* 位置传感器更新周期（秒）= TIM6 1ms */
#define PRINT_PERIOD_MS    10       /* 串口打印周期（ms），在主循环执行 */

static PosSensor_Handle_t g_pos_sensor;   /* 位置传感器设备句柄 */
static volatile uint8_t  g_print_flag = 0;   /* 主循环打印标志 */
static uint16_t          g_print_cnt  = 0;   /* 打印周期计数 */

/* 位置传感器初始化 */
static void User_PosSensor_Init(void)
{
    PosSensor_Init(&g_pos_sensor, POS_SENSOR_SSI, MOTOR_POLE_PAIRS);
}

/* 位置传感器周期采样（TIM6 中断内调用，保持微秒级，不阻塞 SPI 帧） */
static void User_PosSensor_Update(void)
{
    PosSensor_Update(&g_pos_sensor, SENSOR_TICK_S);
}

/* 位置传感器打印（主循环执行；%f 格式化较慢，不能放中断里）
   FireWater：纯数值逗号分隔 + 换行结尾，解析为通道：
   ch0=Mech(机械角度,rad) ch1=Elec(电角度,rad) ch2=Rpm(转速) ch3=Pos(绝对位置,rad)
   ch4=TIM2原始计数（临时诊断，验证 ABZ 后删除） */
static void User_PosSensor_Print(void)
{
    uart_printf("%.3f,%.3f,%.1f,%.3f,%d\n",
                PosSensor_GetMechAngle(&g_pos_sensor),
                PosSensor_GetElecAngle(&g_pos_sensor),
                PosSensor_GetSpeedRpm(&g_pos_sensor),
                PosSensor_GetPosition(&g_pos_sensor),
                (int)MT6701_Abz_GetCount());
}

/* ===================== 芯片中断回调函数 ===================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Led_Loop();                 /* 1ms 呼吸灯 */

        User_PosSensor_Update();    /* 1ms 采样一次位置传感器 */

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

    Led_Init();                     /* 呼吸灯 PWM */

    User_PosSensor_Init();          /* 位置传感器（SSI） */

    HAL_TIM_Base_Start_IT(&htim6);  /* 启动 1ms 定时中断 */
}

void User_Loop(void)
{
    if (g_print_flag)               /* 主循环里打印，避开中断 */
    {
        g_print_flag = 0;
        User_PosSensor_Print();
    }
}
