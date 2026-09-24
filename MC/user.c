#include "user.h"
#include "user_debug.h"
#include "led.h"
#include "tim.h"
#include "usart.h"
#include "mc_db.h"
#include "control.h"
#include "pos_sensor.h"
#include "current_sensor.h"

#define SENSOR_FREQ_HZ    1000     /* 位置传感器更新频率（Hz）= TIM6 1ms */
#define PRINT_PERIOD_MS    20       /* 串口打印周期（ms）：14 通道 CSV 一行约 100 字节，
                                       115200 8N1 下发一行要 ~8.8ms → 周期 10ms 会把串口占满、
                                       偶发丢行；20ms（50Hz）留够余量。
                                       想更快看波形：把波特率提到 460800/921600 再把这里改小 */

/* ===================== 调试开关（只改这里，不用动代码） =====================
 *   USER_PRINT_CSV  1 = 周期打印 14 通道 CSV（FireWater 上位机）；0 = 关闭
 *   TEST_MODE       上电校准链跑完后自动进入的测试：
 *                     0 = 不测（停在 STOP）      1 = 全开环（VVVF）
 *                     2 = 半开环（角度闭环+电压开环）  3 = 电流环（Id/Iq 闭环）
 *   TEST_IQ_A       电流环测试的 Iq 给定（A）：首次先用小值（0.1~0.2）试，
 *                   确认符号/方向没问题再提到额定 0.5A
 *   两者的参数都在 User_Test_Setup() 里对应分支内，改参数不用改结构 */
#define USER_PRINT_CSV     1        /* 电流环测试需要看 Id/Iq/Ud/Uq → 打开 */
#define TEST_MODE          3        /* 3 = 电流环 */
#define TEST_IQ_A          0.5f     /* 电流环测试给定（A）；额定 0.5A */

static Led_Handle_t         g_led_G;          /* 绿色 LED 设备句柄 */
static volatile uint8_t  g_print_flag = 0;   /* 主循环打印标志 */
static uint16_t          g_print_cnt  = 0;   /* 打印周期计数 */
static volatile uint8_t  g_led_tick   = 0;   /* LED 1ms 节拍标志（中断置位、主循环消费） */
static volatile uint8_t  g_cfg_report = 0;   /* 测试配置一次性打印标志（确认烧进去的是哪套参数） */

/* ===================== 芯片中断回调函数 ===================== */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        /* 观测：进中断拉高 / 出中断拉低（TEST3 = PB3）
           脉冲宽度 = 注入中断处理耗时，示波器可据此测量 */
        TEST3(1);

        /* 20kHz 控制调度：一个分频调度器驱动高/中/低三环（注入 mc 实例） */
        Control_Sched(&mc);

        TEST3(0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* LED 状态机不在这里跑了：它一次 70+ 条指令，还含 4 个运行期整数除法
           （M4 没有硬件除法，编译成库调用），而 LED 视觉上 1ms 的抖动无关紧要。
           中断里只置标志，实际更新挪到主循环（见 User_Loop） */
        g_led_tick = 1u;

        if (++g_print_cnt >= PRINT_PERIOD_MS)   /* 每 10ms 置打印标志 */
        {
            g_print_cnt  = 0;
            g_print_flag = 1;
        }
    }
}

/* 串口接收：当前只发送（Debug_Usart_Init 不再开接收中断），
   接收命令后期开发，届时在此处注册 HAL_UART_RxCpltCallback */

/* ===================== 初始化与循环 ===================== */
void User_Init(void)
{
    /* ================= 系统级性能/抖动设置（放在这里，避免被 CubeMX 重新生成覆盖） =================
     * 1) 中断优先级：控制环（ADC1 注入转换回调 = 20kHz FOC）必须最高；
     *    通信类（SPI3-RX DMA = 编码器、USART1 + 其 TX DMA = 打印）降一级。
     *    原来它们和 ADC 同为优先级 0：同优先级不能互相抢占，于是编码器 DMA 回调里
     *    那 18 次逐位 CRC 循环（~0.75us）会原样变成控制环的抖动。
     *    降级后控制环抖动≈0，代价只是编码器帧结束晚几微秒（无害）。
     * 2) Flash 预取：170MHz + 4 等待周期下 ST 建议开启；本工程 ICACHE/DCACHE 是复位默认
     *    开着的，只有预取在 stm32g4xx_hal_conf.h 里被设成 PREFETCH_ENABLE=0。
     *    HAL 在 Flash 擦写时会自行关闭再恢复，所以在用户代码里补一句是安全的。 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn,        0, 0);   /* 控制环：最高 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);   /* 编码器 SPI3-RX DMA */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);   /* 打印 USART1-TX DMA */
    HAL_NVIC_SetPriority(USART1_IRQn,        1, 0);   /* 打印 USART1 */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    /* ----- LED0 绑定 BSP 通道 0（BSP 绑定在设备层内部完成） ----- */
    Led_Init(&g_led_G, 0);    

    /* ----- 调试串口（含接收中断） ----- */
    Debug_Usart_Init();

    /* ----- 启动 1ms 定时中断 ----- */
    HAL_TIM_Base_Start_IT(&htim6);

    /* ----- LED0 开始呼吸 ----- */
    Led_SetBreathe(&g_led_G, 255, 50, 2500, LED_WAVE_SINE);

    /* ----- 控制层：设备初始化（PWM/位置/电流，句柄写入 mc.device） -----
       模式为 Database 共享数据 mc.mode（初始停止），直接切换 */
    Control_Init(&mc);
}

/* ===================== 测试入口（调试用，开发完成后移除） =====================
   选哪个测试看文件顶部 TEST_MODE；每个分支 = 一组参数 + 一个模式。
   判断规则：
   1. 上电校准链（电流校准 → 编码器校准 → 停止）执行期间不干预模式；
   2. 仅当系统就绪（当前停止 + 电流校准完成 + 编码器已校准）才允许切入测试模式 */
static void User_Test_Setup(void)
{
    static uint8_t cfg_once = 0u;   /* 配置只上报一次 */

    if (mc.mode != MODE_STOP || !mc.device.cur.calib_done || !mc.enc_calib_done) {
        return;     /* 未就绪：不动模式（校准链自己在跑） */
    }
    if (mc.oc_latched != 0u) {
        return;     /* 已触发过流保护：不再自动切入测试（重新上电才清） */
    }

    if (cfg_once == 0u) {           /* 首次就绪：打印一次实际生效的配置 */
        cfg_once     = 1u;
        g_cfg_report = 1u;
    }

#if (TEST_MODE == 1)
    /* ---- 全开环（电压/频率开环驱动）+ 编码器校准验证观测 ----
       用实测参数（R=4.2Ω、ψf=0.0037Wb，母线 12V）折算的量级参考：
         0.05 调制比 = 0.3V 相电压 → 堵转电流 ≈0.07A；空载转速 ≈110rpm（很慢）
         0.30 调制比 = 1.8V        → 堵转电流 ≈0.43A（接近额定）
       注：早期注解"0.3 曾致 3.x A"不对（当时电流 bias 没采对，读数≈常数 3.3A） */
    mc.target.ol_volt = 0.05f;      /* Uq 电压幅值（调制比） */
    mc.target.ol_freq = 5.0f;       /* 电频率 (Hz) */
    mc.mode = MODE_FULL_OPEN_LOOP;

#elif (TEST_MODE == 2)
    /* ---- 半开环（角度闭环 + 电流开环） ----
       角度已带方向（两点锁轴校准自动定 dir），映射正确时能自持旋转；
       若转不起来：先看 ch7/ch8——
         |Id| 有值且 Iq≈0 → 角度仍不对（方向判据被标 SUSPECT，见 CAL 行）；
         Iq 有值 → 对轴正确；此时若电机不转只是"电压不够/机械卡"，可小步上调 uq_target。
       注意：转子是自由的时，0.04 调制比(0.24V) 就会把它带到 ~100rpm 空载转速，
       电流随之掉到摩擦值（≈0.06A）——看着像"没电流/堵转"，其实在转。
       单位：ud/uq_target 是调制比（1 = Udc/2），不是伏特 */
    mc.target.ud_target = 0.0f;     /* Ud 给定（调制比，通常 0） */
    mc.target.uq_target = 0.04f;    /* Uq 给定（调制比） */
    mc.mode = MODE_HALF_OPEN_LOOP;

#elif (TEST_MODE == 3)
    /* ---- 电流环（角度闭环 + 电流闭环）----
       测试要点：
       1) 先确认上电 CAL 行是 `dir=… ok`（角度/方向必须已校准，电流环用它做 Park）；
       2) 必须把轴**夹死**再看稳态——转子自由时电流环会把电机加速到电压顶速
          （空载顶速 = (限幅×Udc/2 − R·I)/ψf，12V 下 ≈2280rpm），
          电流只剩摩擦值，看着"跟踪不上"其实是环已经饱和；
       3) 看 ch7/ch8（Id/Iq，A）与 ch9/ch10（Ud/Uq，调制比）：
          期望 Iq ≈ +TEST_IQ_A、Id ≈ 0；堵转稳态 Uq ≈ R·Iq/(Udc/2)
          ✓已实测（母线 12V）：R=3.5Ω、0.5A → Uq=0.292（实测 Iq=0.508 / Uq=0.292、Id=0.000）；
       4) 若立刻出现 `FAULT: OC trip` → 电流反馈符号反了：
          把 drv8311.h 的 DRV8311_I_SIGN 改成 -1.0f，重新上电再试 */
    mc.target.id_target = 0.0f;         /* Id 目标（A，通常 0） */
    mc.target.iq_target = TEST_IQ_A;    /* Iq 目标（A，额定 0.5A） */
    mc.mode = MODE_CURRENT_LOOP;
#endif
}

void User_Loop(void)
{
    /* LED 状态机（1ms 节拍，由 TIM6 中断置标志）：放在主循环，
       不在中断里做除法和那 70+ 条指令 —— LED 的毫秒级抖动肉眼看不出来 */
    if (g_led_tick)
    {
        g_led_tick = 0u;
        Led_Update(&g_led_G, SENSOR_FREQ_HZ);
    }

    /* 测试入口：模式与参数设置（带就绪判断，不打断上电校准链） */
    User_Test_Setup();

    /* 编码器校准结果一次性上报（两点锁轴：零点 + 方向），人类可读单行，与周期 CSV 区分：
       dir=-1 表示编码器计数方向与电机相序相反，已在角度映射里自动纠正——
       这正是"锁轴对齐做了、半开环却不转"的原因；
       SUSPECT 表示第二点转子几乎没走动，方向按 +1 兜底、结果不可信
       （查锁轴电压/机械是否卡死/编码器角度是否在更新）。
       串口忙则本轮不清标志、下一轮重试，避免与周期打印撞车丢消息 */
    if (mc.calib_report)
    {
        if (uart_printf("CAL: off=%.4f dir=%+d dmech=%.4f expect=%.4f %s\n",
                        mc.device.pos.elec_offset,
                        (int)mc.device.pos.elec_dir,
                        mc.enc_calib.dmech,
                        (3.141592653589793f * 0.5f) / (float)MOTOR_POLE_PAIRS,
                        mc.enc_calib.suspect ? "SUSPECT(rotor-not-moved)"
                                             : "ok"))
        {
            mc.calib_report = 0u;
        }
    }

    /* 本次测试配置一次性上报：确认烧进去的到底是哪套参数（改宏后忘了重新下载时一眼可见） */
    if (g_cfg_report)
    {
        if (uart_printf("CFG: TEST_MODE=%d iq*=%.2fA id*=%.2fA | umax=%.2f BW=%.0fHz FF=%d "
                        "kp=%.3f ki=%.0f | R=%.2fohm L=%.2fmH psi=%.4fWb Udc_nom=%.1fV\n",
                        (int)TEST_MODE, mc.target.iq_target, mc.target.id_target,
                        mc.current.u_max, CURRENT_BW_HZ, (int)mc.current.ff_en,
                        mc.current.kp, mc.current.ki,
                        MOTOR_R_PHASE, MOTOR_LD * 1000.0f, MOTOR_FLUX, MOTOR_UDC_NOMINAL))
        {
            g_cfg_report = 0u;
        }
    }

    /* 故障一次性上报（如电流环过流保护触发）：与周期 CSV 区分，串口忙则下轮重试 */
    if (mc.fault_report)
    {
        if (uart_printf("FAULT: OC trip, |I|>%.1fA -> STOP (power-cycle to re-arm)\n",
                        CURRENT_I_TRIP))
        {
            mc.fault_report = 0u;
        }
    }

    /* 驱动器故障引脚（nFAULT）：故障时芯片直接关断输出级——这是"给了电压却不走电流"
       的常见原因，原固件完全看不到它。状态变化时打印一次（故障消失后可再报） */
    {
        static uint8_t fault_latched = 0u;
        uint8_t nfault = MotorPwm_FaultRead(&mc.device.pwm);

        if (nfault != 0u) {
            if (fault_latched == 0u &&
                uart_printf("FAULT: DRV8311 nFAULT asserted (PC6 low) - output stage off\n")) {
                fault_latched = 1u;
            }
        } else {
            fault_latched = 0u;
        }
    }

    /* 周期打印：FireWater 纯数值逗号分隔 + 换行结尾
       ch0=Mech(rad) ch1=Elec(rad) ch2=Rpm ch3=Pos(rad)
       ch4=Ia(A) ch5=Ib(A) ch6=Ic(A)
       ch7=Id(A) ch8=Iq(A) ch9=Ud ch10=Uq（电流环反馈/输出）
       ch11=Du ch12=Dv ch13=Dw（最后写入 PWM 的三相占空比，读设备句柄缓存）
       坐标系：ch0~ch3 全部为**电机坐标系**（已含 elec_dir），正方向 = 电角度增大方向，
       因此 ch0 与 ch1 一定同向（各绕 1 圈 / 7 圈）；编码器原始机械角（可能与 ch1 反向）
       要看得用 PosSensor_GetMechAngle()，上电 CAL 行会给出 dir 与两点位移
       （串口接收命令后期开发，当前只发送） */
    if (g_print_flag)
    {
        g_print_flag = 0;

#if (USER_PRINT_CSV)
        PosSensor_Handle_t     *pos = &mc.device.pos;   /* 设备句柄由 db 统一管理 */
        CurrentSensor_Handle_t *cur = &mc.device.cur;
        MotorPwm_Handle_t      *pwm = &mc.device.pwm;

        uart_printf("%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    PosSensor_GetMechAngleMotor(pos),
                    PosSensor_GetElecAngle(pos),
                    PosSensor_GetSpeedRpm(pos),
                    PosSensor_GetPosition(pos),
                    CurrentSensor_GetIa(cur),
                    CurrentSensor_GetIb(cur),
                    CurrentSensor_GetIc(cur),
                    mc.state.id,
                    mc.state.iq,
                    mc.state.ud,
                    mc.state.uq,
                    pwm->duty[0],
                    pwm->duty[1],
                    pwm->duty[2]);
#endif
    }
}
