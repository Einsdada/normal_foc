#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "motor_pwm.h"
#include "current_sensor.h"
#include "tim.h"
#include "adc.h"
#include "pos_sensor.h"

#define MOTOR_POLE_PAIRS   7        /* 电机极对数，按实际电机修改 */
#define SENSOR_FREQ_HZ    1000     /* 位置传感器更新频率（Hz）= TIM6 1ms */
#define PRINT_PERIOD_MS    10       /* 串口打印周期（ms），在主循环执行 */

static PosSensor_Handle_t    g_pos_sensor;    /* 位置传感器设备句柄 */
static Led_Handle_t         g_led_G;          /* 绿色 LED 设备句柄 */
static MotorPwm_Handle_t    g_motor_pwm;      /* 电机 PWM 设备句柄 */
static CurrentSensor_Handle_t g_cur;          /* 三相电流传感设备句柄 */
static volatile uint8_t  g_print_flag = 0;   /* 主循环打印标志 */
static uint16_t          g_print_cnt  = 0;   /* 打印周期计数 */
static uint8_t           g_bias_printed = 0; /* 校准完成诊断只打印一次 */

/* ===================== 芯片中断回调函数 ===================== */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        /* 观测：进中断拉高 / 出中断拉低（TEST3 = PB3）
           脉冲宽度 = 注入中断处理耗时，示波器可据此测量 */
        TEST3(1);

        /* 校准分流（单函数合成式）：校准未完成 → 逐次累加零点码（JDR 必然有效）；
           校准完成 → 正常换算更新（每相 1 次整型减法 + 1 次乘法） */
        if (!g_cur.calib_done)
            CurrentSensor_Calib(&g_cur);
        else
            CurrentSensor_Update(&g_cur);

        /* PWM 输出：开环固定占空比（归一化 0.0~1.0，设备层内部映射到 ARR）
           40kHz 电流环位置，后续替换为 FOC 计算结果 */
        MotorPwm_SetDuty(&g_motor_pwm, 0.3f, 0.5f, 0.7f);

        TEST3(0);
    }
}

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
    /* ----- 电机 PWM（设备层，内部绑定 BSP 并启动三路载波，0% 占空比） ----- */
    MotorPwm_Init(&g_motor_pwm);

    /* ----- 位置传感器（跳线帽当前接 SSI 模式） ----- */
    PosSensor_Init(&g_pos_sensor, POS_SENSOR_SSI, MOTOR_POLE_PAIRS);

    /* ----- 三相电流采集（设备层，内部绑定 BSP：ADC 校准 + CC4 触发源 + 注入转换） ----- */
    CurrentSensor_Init(&g_cur, CURRENT_SENSOR_3PH);

    /* ----- LED0 绑定 BSP 通道 0（BSP 绑定在设备层内部完成） ----- */
    Led_Init(&g_led_G, 0);    

    /* ----- 调试串口 ----- */
    Debug_Usart_Init();

    /* ----- 启动 1ms 定时中断 ----- */
    HAL_TIM_Base_Start_IT(&htim6);

    /* ----- 使能驱动器输出（sleep 拉高）——DRV8311 才能向电机输出 ----- */  
       /* 注意：使能后注入中断里设置的占空比会直接驱动电机，
       先保持占空比为 0 或用示波器确认驱动输出后再加电压 */
    MotorPwm_Enable(&g_motor_pwm);

    /* ----- LED0 开始呼吸 ----- */
    Led_SetBreathe(&g_led_G, 255, 50, 2500, LED_WAVE_SINE);

}

void User_Loop(void)
{
    /* 校准完成诊断：只在完成瞬间打印一次（正常应 ≈2018/2048；若为 0 说明校准链路异常） */
    if (g_cur.calib_done && !g_bias_printed)
    {
        g_bias_printed = 1;
        uart_printf("calib bias=%d,%d,%d\r\n",
                    (int)g_cur.bias[0], (int)g_cur.bias[1], (int)g_cur.bias[2]);
    }

    if (g_print_flag)               /* 主循环里打印，避开中断（%f 较慢） */
    {
        g_print_flag = 0;
        /* FireWater：纯数值逗号分隔 + 换行结尾
           ch0=Mech(rad) ch1=Elec(rad) ch2=Rpm ch3=Pos(rad)
           ch4=Ia(A) ch5=Ib(A) ch6=Ic(A) */
        uart_printf("%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f\n",
                    PosSensor_GetMechAngle(&g_pos_sensor),
                    PosSensor_GetElecAngle(&g_pos_sensor),
                    PosSensor_GetSpeedRpm(&g_pos_sensor),
                    PosSensor_GetPosition(&g_pos_sensor),
                    CurrentSensor_GetIa(&g_cur),
                    CurrentSensor_GetIb(&g_cur),
                    CurrentSensor_GetIc(&g_cur));
    }
} 
