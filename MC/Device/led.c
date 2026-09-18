#include "led.h"
#include "led_bsp.h"   /* 设备层内部绑定 BSP 亮度函数 */

/* =====================================================================
 * LED 设备层实现（单色 LED，内部绑定 BSP）
 * - 设备层只写"点亮方式"：模式状态机 + 亮度/波形计算 + 伽马校正
 * - BSP 亮度函数在 Led_Init 内部绑定（函数指针），输出经 set_bright 直达底层
 * - 伽马查找表：亮度 0~255 → 占空比 0~DUTY_MAX（gamma=2.2 预生成）
 * - 正弦半波查找表：呼吸正弦波形（64 点，0~255）
 * ===================================================================== */

/* ---------- 伽马校正查找表（gamma=2.2） ----------
 * 索引 = 亮度 0~255，值 = 占空比 0~1000
 * 公式 round((i/255)^2.2 * 1000) 预生成：
 * 低亮度区间映射更密，使人眼感知的亮度随数值近似线性
 * 运行时查表，零浮点计算 */
static const uint16_t gamma_tbl[256] = {
       0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    2,    2,
       2,    3,    3,    3,    4,    4,    5,    5,    6,    6,    7,    7,    8,    8,    9,   10,
      10,   11,   12,   13,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,   24,
      25,   27,   28,   29,   30,   32,   33,   34,   36,   37,   38,   40,   41,   43,   45,   46,
      48,   49,   51,   53,   55,   56,   58,   60,   62,   64,   66,   68,   70,   72,   74,   76,
      78,   80,   82,   85,   87,   89,   92,   94,   96,   99,  101,  104,  106,  109,  111,  114,
     117,  119,  122,  125,  128,  130,  133,  136,  139,  142,  145,  148,  151,  154,  157,  160,
     164,  167,  170,  173,  177,  180,  184,  187,  190,  194,  198,  201,  205,  208,  212,  216,
     220,  223,  227,  231,  235,  239,  243,  247,  251,  255,  259,  263,  267,  272,  276,  280,
     284,  289,  293,  298,  302,  307,  311,  316,  320,  325,  330,  334,  339,  344,  349,  354,
     359,  364,  369,  374,  379,  384,  389,  394,  399,  405,  410,  415,  421,  426,  431,  437,
     442,  448,  453,  459,  465,  470,  476,  482,  488,  494,  500,  505,  511,  517,  523,  530,
     536,  542,  548,  554,  560,  567,  573,  580,  586,  592,  599,  605,  612,  619,  625,  632,
     639,  646,  652,  659,  666,  673,  680,  687,  694,  701,  708,  715,  723,  730,  737,  745,
     752,  759,  767,  774,  782,  789,  797,  805,  812,  820,  828,  836,  843,  851,  859,  867,
     875,  883,  891,  899,  908,  916,  924,  932,  941,  949,  957,  966,  974,  983,  991, 1000,
};

/* ---------- 正弦半波查找表（sin(π·i/63)·255，i=0~63） ----------
 * 半个正弦周期 0~π 归一化到 0~255，用于呼吸波形
 * 峰值在中点（idx=32），两端为 0，配合 bright_min~bright_max 插值 */
static const uint8_t sin_half_tbl[64] = {
       0,   13,   25,   38,   50,   63,   75,   87,   99,  111,  122,  133,  144,  154,  164,  173,
     183,  191,  199,  207,  214,  221,  227,  233,  237,  242,  245,  249,  251,  253,  254,  255,
     255,  255,  254,  253,  251,  249,  245,  242,  237,  233,  227,  221,  214,  207,  199,  191,
     183,  173,  164,  154,  144,  133,  122,  111,   99,   87,   75,   63,   50,   38,   25,   13,
};

/* ---------- 输出亮度到 BSP（设备层唯一对底层的调用点） ----------
 * @param handle     LED 句柄（含 BSP 通道号与亮度函数指针）
 * @param bright     亮度 0~255（伽马校正前的线性亮度）
 * @note  通过句柄注入的 set_bright 输出，设备层不感知 PWM/GPIO 驱动方式 */
static void Led_Output(Led_Handle_t *handle, uint8_t bright)
{
    if (handle->set_bright)
        handle->set_bright(handle->ch, gamma_tbl[bright]);
}

/* ---------- 单帧亮度计算（点亮方式状态机） ----------
 * @param  handle  LED 句柄
 * @retval 本帧应输出的线性亮度 0~255
 * @note  按当前模式与周期计数 cnt_ms 计算：
 *        OFF=0；ON=最大亮度；BLINK=方波（周期前半亮/后半灭）；
 *        BREATHE=三角波或正弦波在 min~max 间渐变 */
static uint8_t Led_CalcBright(Led_Handle_t *handle)
{
    uint32_t phase;

    switch (handle->mode)
    {
    case LED_MODE_OFF:
        return 0;

    case LED_MODE_ON:
        return handle->bright;

    case LED_MODE_BLINK:                    /* 方波：周期内按 duty 占比亮/灭 */
    {
        uint32_t on_ms = (uint32_t)handle->period_ms * handle->duty / 100U;
        return (handle->cnt_ms % handle->period_ms) < on_ms ? handle->bright : 0;
    }

    case LED_MODE_BREATHE: 
    {
        uint16_t range = (uint16_t)(handle->bright_max - handle->bright_min);
        phase = handle->cnt_ms % handle->period_ms;

        if (handle->wave == LED_WAVE_SINE)  /* 正弦波 */
        {
            uint8_t idx = (uint8_t)((uint32_t)phase * 64U / handle->period_ms);
            return (uint8_t)(handle->bright_min + (uint16_t)range * sin_half_tbl[idx] / 255U);
        }
        else                                /* 三角波 */
        {
            uint16_t half = handle->period_ms / 2U;
            if (phase < half)
                return (uint8_t)(handle->bright_min + (uint32_t)range * phase / half);
            return (uint8_t)(handle->bright_max - (uint32_t)range * (phase - half) / half);
        }
    }
    default:
        return 0;
    }
}

/* ---------- 对外接口 ---------- */

static uint8_t g_bsp_inited = 0;    /* BSP 已初始化标志（多实例只启动一次） */

/**
 * @brief  初始化 LED：绑定 BSP 通道与亮度函数，初始化为熄灭
 * @param  handle  LED 句柄
 * @param  ch      BSP 通道号（0 ~ CH_COUNT-1）
 * @note   BSP 亮度函数在设备层内部绑定为 Led_Bsp_SetDuty，上层无需关心
 *         首次调用时启动全部 BSP 通道（PWM 输出），多实例只启动一次
 *         默认参数：常亮模式、最大亮度 255、周期 1000ms、三角波
 */
void Led_Init(Led_Handle_t *handle, uint8_t ch)
{
    if (!g_bsp_inited)
    {
        Led_Bsp_Init();             /* 启动全部 BSP 通道（PWM Start / GPIO 置灭） */
        g_bsp_inited = 1;
    }

    handle->ch          = ch;
    handle->set_bright  = Led_Bsp_SetDuty;  /* 设备层内部绑定 BSP 亮度接口 */
    handle->mode        = LED_MODE_OFF;
    handle->bright      = 255;              /* 常亮亮度 / 闪烁亮电平默认最亮 */
    handle->bright_max  = 255;              /* 呼吸峰值默认最亮 */
    handle->bright_min  = 0;                /* 呼吸谷值默认熄灭 */
    handle->duty        = 50;               /* 闪烁点亮占比默认 50% */
    handle->period_ms   = 1000;
    handle->wave        = LED_WAVE_TRIANGLE;
    handle->cnt_ms      = 0;
    Led_Output(handle, 0);                  /* 初始熄灭 */
}

/**
 * @brief  周期更新 LED 状态机并刷新输出
 * @param  handle  LED 句柄
 * @param  freq    本函数调用频率（Hz），如 1ms 中断调用传 1000
 * @note   内部按 1000/freq 推进毫秒计数，支持闪烁/呼吸周期换算
 */
void Led_Update(Led_Handle_t *handle, uint16_t freq)
{
    if (freq == 0) freq = 1;                    /* 防除零 */
    handle->cnt_ms += 1000U / freq;             /* 每次调用推进的毫秒数 */
    Led_Output(handle, Led_CalcBright(handle));
}

/**
 * @brief  常亮：以指定亮度持续点亮
 * @param  handle  LED 句柄
 * @param  bright  亮度 0~255（0 等于熄灭）
 */
void Led_SetOn(Led_Handle_t *handle, uint8_t bright)
{
    handle->mode   = LED_MODE_ON;
    handle->bright = bright;
    handle->cnt_ms = 0;
    Led_Output(handle, Led_CalcBright(handle));     /* 立即生效 */
}

/**
 * @brief  闪烁：以指定亮度按周期方波亮灭
 * @param  handle      LED 句柄
 * @param  bright      亮电平亮度 0~255
 * @param  period_ms   闪烁周期（ms，亮灭一个完整循环；0 自动按 1 处理）
 * @param  duty        点亮时长占比 0~100（%），如 50=亮灭各半、30=亮 30% 灭 70%
 * @note   duty 超 100 自动截断；修改后周期重新起算并立即输出一帧
 */
void Led_SetBlink(Led_Handle_t *handle, uint8_t bright,
                  uint16_t period_ms, uint8_t duty)
{
    handle->mode      = LED_MODE_BLINK;
    handle->bright    = bright;
    handle->period_ms = period_ms;
    handle->duty      = (duty > 100) ? 100 : duty;  /* 占比截断到 0~100 */
    handle->cnt_ms    = 0;                          /* 周期从新参数重新起算 */
    if (handle->period_ms == 0) handle->period_ms = 1;  /* 防除零 */
    Led_Output(handle, Led_CalcBright(handle));     /* 立即生效 */
}

/**
 * @brief  呼吸：亮度在 [bright_min, bright_max] 区间按波形渐变
 * @param  handle      LED 句柄
 * @param  bright_max  峰值亮度 0~255
 * @param  bright_min  谷值亮度 0~255（须 ≤ bright_max）
 * @param  period_ms   呼吸周期（ms，一次渐亮到渐暗；0 自动按 1 处理）
 * @param  wave        波形 Led_Wave_e（TRIANGLE 线性 / SINE 平滑）
 * @note   bright_min > bright_max 时自动交换；修改后周期重新起算并立即输出一帧
 */
void Led_SetBreathe(Led_Handle_t *handle, uint8_t bright_max,
                    uint8_t bright_min, uint16_t period_ms, uint8_t wave)
{
    if (bright_min > bright_max)            /* 防呆：谷值大于峰值时交换 */
    {
        uint8_t tmp   = bright_max;
        bright_max    = bright_min;
        bright_min    = tmp;
    }
    handle->mode       = LED_MODE_BREATHE;
    handle->bright_max = bright_max;
    handle->bright_min = bright_min;
    handle->period_ms  = period_ms;
    handle->wave       = wave;
    handle->cnt_ms     = 0;                 /* 周期从新参数重新起算 */
    if (handle->period_ms == 0) handle->period_ms = 1;  /* 防除零 */
    Led_Output(handle, Led_CalcBright(handle));         /* 立即生效 */
}

/**
 * @brief  熄灭 LED（切到 OFF 模式并立即输出 0）
 * @param  handle  LED 句柄
 */
void Led_Off(Led_Handle_t *handle)
{
    handle->mode   = LED_MODE_OFF;
    handle->cnt_ms = 0;
    Led_Output(handle, 0);
}
