#include "drv8311.h"
#include "main.h"       /* Motor_sleep_Pin */
#include "tim.h"        /* htim1 与 TIM_CHANNEL_* */

/* =====================================================================
 * DRV8311 PWM 输出实现（三线：TIM1 CH1/CH2/CH3 → PA8/PA9/PA10）
 * - 三路占空比独立设置，接口统一归一化 0.0~1.0
 * - 占空比内部映射到 TIM1 ARR（4250，20kHz 中心对齐）
 * - 驱动器使能/关断走 sleep 引脚（PA11）：高=工作，低=三相关断
 * ===================================================================== */

/* ---------- 设置三相占空比（归一化 0.0~1.0，直接写三路 CCR） ----------
 * 这是 20kHz 中断里每拍都要跑的路径，写得尽量"裸"：
 *  1. 直接写 TIM1->CCR1/2/3，不用 __HAL_TIM_SET_COMPARE 宏 —— 那个宏每次都要
 *     用 channel 值算一次 CCR 寄存器地址（移位+加）并解引用 htim1.Instance，
 *     三通道共多花约 15 条指令；drv8311_cur.c 写 CCR4 也是直接写，风格一致
 *  2. 去掉通道数组与循环：三次 FMUL + VCVT + STR，约 10 条指令
 *  3. 不再重复限幅：SVPWM（foc.c）输出已钳到 [0,1]，设备层 MotorPwm_SetDuty
 *     又钳一次，这里是第三层 —— 去掉省 6 个分支。
 *     ⚠ 调用方必须保证 a/b/c ∈ [0.0, 1.0]（见 drv8311.h 的接口约定） */
void Drv8311_Pwm_SetDuty(float a, float b, float c)
{
    TIM1->CCR1 = (uint32_t)(a * (float)DRV8311_PWM_ARR);   /* A 相 → PA8  */
    TIM1->CCR2 = (uint32_t)(b * (float)DRV8311_PWM_ARR);   /* B 相 → PA9  */
    TIM1->CCR3 = (uint32_t)(c * (float)DRV8311_PWM_ARR);   /* C 相 → PA10 */
}

/* ---------- 启动三路 PWM（0% 占空比载波） ---------- */
void Drv8311_Pwm_Init(void)
{
    /* 驱动芯片模式（PMODE，PA12）：拉高 = 3x PWM 模式——
       3 个 PWM 输入（INHA/INHB/INHC），芯片内部生成互补与死区；
       CubeMX 初始化时该引脚为低（1x PWM：PWM+DIR+BRK），必须在此拉高 */
    Motor_mode_GPIO_Port->BSRR = Motor_mode_Pin;

    /* 3x PWM 模式下低边输入（INHLA/INHLB/INHLC = PB13/14/15）保持高电平：
       DRV8311 低侧输入需拉高（低边由芯片内部死区逻辑控制）；
       CubeMX 中已设为 GPIO 输出，此处显式拉高保证不依赖生成文件 */
    Motor_PWM_AN_GPIO_Port->BSRR =
        (uint32_t)Motor_PWM_AN_Pin | (uint32_t)Motor_PWM_BN_Pin | (uint32_t)Motor_PWM_CN_Pin;

    /* 故障引脚 nFAULT（PC6）：开漏输出，CubeMX 配的是 NOPULL 输入（悬空读不准），
       这里改成带内部上拉的输入，才能读到"有故障 = 低" */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin  = Motro_fault_Pin;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(Motro_fault_GPIO_Port, &gpio);
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8 → DRV8311 A 相输入 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* PA9 → B 相 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* PA10 → C 相 */
    /* 注：不输出互补 N（DRV8311 集成门驱动，内部生成互补与死区） */
}

/* ---------- 读故障引脚（1 = 有故障；nFAULT 开漏低有效） ---------- */
uint8_t Drv8311_FaultRead(void)
{
    return (HAL_GPIO_ReadPin(Motro_fault_GPIO_Port, Motro_fault_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

/* ---------- 使能/关断驱动器输出 ---------- */
void Drv8311_Pwm_Enable(void)
{
    Motor_sleep_GPIO_Port->BSRR = Motor_sleep_Pin;      /* sleep 高，驱动器工作 */
}

void Drv8311_Pwm_Disable(void)
{
    Motor_sleep_GPIO_Port->BSRR = (uint32_t)Motor_sleep_Pin << 16U;   /* sleep 低，三相关断 */
}
