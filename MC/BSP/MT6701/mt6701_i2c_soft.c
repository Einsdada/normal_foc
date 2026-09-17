#include "mt6701.h"
#include "main.h"

/* =====================================================================
 * MT6701 软件 I2C 配置（不依赖 HAL I2C，不改 CubeMX）
 * 用途：上电时通过 I2C 把芯片 ABZ 分辨率写入 RAM 配置
 *       （EEPROM 烧写要求 VDD>4.5V，3.3V 供电只能写 RAM，断电失效，
 *         故每次上电由 MCU 自动配置一次）
 * 引脚复用：A(PA0)=SDA、B(PA1)=SCL；运行时临时切 GPIO，配置完恢复 TIM2 AF
 * ===================================================================== */

/* ---- I2C 引脚（芯片 A=SDA、B=SCL） ---- */
#define I2C_SDA_PIN   Encoder_A_Pin   /* PA0 */
#define I2C_SCL_PIN   Encoder_B_Pin   /* PA1 */

/* ---- 寄存器级 I2C 操作 ---- */
#define I2C_SDA_HI()  (GPIOA->BSRR = I2C_SDA_PIN)
#define I2C_SDA_LO()  (GPIOA->BSRR = (uint32_t)I2C_SDA_PIN << 16U)
#define I2C_SCL_HI()  (GPIOA->BSRR = I2C_SCL_PIN)
#define I2C_SCL_LO()  (GPIOA->BSRR = (uint32_t)I2C_SCL_PIN << 16U)
#define I2C_SDA_GET() ((GPIOA->IDR & I2C_SDA_PIN) != 0U)

/* 100kHz：@170MHz 每半周期约 5µs */
static void i2c_delay(void)
{
    volatile uint32_t n = 100U;
    while (n--) { __NOP(); }
}

static void i2c_start(void)
{
    I2C_SDA_HI(); I2C_SCL_HI(); i2c_delay();
    I2C_SDA_LO();               i2c_delay();
    I2C_SCL_LO();               i2c_delay();
}

static void i2c_stop(void)
{
    I2C_SDA_LO();               i2c_delay();
    I2C_SCL_HI();               i2c_delay();
    I2C_SDA_HI();               i2c_delay();
}

/* 收从机应答位，返回 1=ACK */
static uint8_t i2c_ack(void)
{
    uint8_t ack;
    I2C_SDA_HI();               i2c_delay();
    I2C_SCL_HI();               i2c_delay();
    ack = !I2C_SDA_GET();
    I2C_SCL_LO();               i2c_delay();
    return ack;
}

/* 发 1 字节，返回 1=从机 ACK */
static uint8_t i2c_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--)
    {
        if (data & (1U << i)) I2C_SDA_HI(); else I2C_SDA_LO();
        i2c_delay();
        I2C_SCL_HI(); i2c_delay();
        I2C_SCL_LO(); i2c_delay();
    }
    return i2c_ack();
}

/* 收 1 字节；ack=1 主机拉低继续读，ack=0 释放结束 */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t data = 0;
    I2C_SDA_HI();
    for (int i = 7; i >= 0; i--)
    {
        I2C_SCL_HI(); i2c_delay();
        data = (data << 1) | (I2C_SDA_GET() ? 1U : 0U);
        I2C_SCL_LO(); i2c_delay();
    }
    if (ack) I2C_SDA_LO(); else I2C_SDA_HI();
    i2c_delay();
    I2C_SCL_HI(); i2c_delay();
    I2C_SCL_LO(); i2c_delay();
    return data;
}

/* ---- 引脚复用临时切换（PA0/PA1/PA5） ---- */
static uint32_t moder_saved, otyper_saved, pupdr_saved, afr_saved;

static void pin_to_gpio_i2c(void)
{
    moder_saved  = GPIOA->MODER;
    otyper_saved = GPIOA->OTYPER;
    pupdr_saved  = GPIOA->PUPDR;
    afr_saved    = GPIOA->AFR[0];

    /* PA0/PA1：GPIO 输出、开漏、内部上拉（I2C SDA/SCL） */
    GPIOA->MODER &= ~((3U << 0) | (3U << 2));
    GPIOA->MODER |=  (1U << 0) | (1U << 2);
    GPIOA->OTYPER |=  (I2C_SDA_PIN | I2C_SCL_PIN);
    GPIOA->PUPDR  &= ~((3U << 0) | (3U << 2));
    GPIOA->PUPDR  |=  (1U << 0) | (1U << 2);

    /* PA5（Z）：GPIO 输出推挽拉高，满足 I2C 模式 Z 为高 */
    GPIOA->MODER  &= ~(3U << 10);
    GPIOA->MODER  |=  (1U << 10);
    GPIOA->OTYPER &= ~Encoder_Z_Pin;
    GPIOA->PUPDR  &= ~(3U << 10);
    GPIOA->BSRR = Encoder_Z_Pin;
}

static void pin_restore(void)
{
    GPIOA->MODER  = moder_saved;
    GPIOA->OTYPER = otyper_saved;
    GPIOA->PUPDR  = pupdr_saved;
    GPIOA->AFR[0] = afr_saved;
}

/* ---- MT6701 寄存器读写（从机地址 0x06） ---- */
static uint8_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t ok = 1;
    i2c_start();
    ok &= i2c_write_byte(0x0C);   /* 地址 + 写 */
    ok &= i2c_write_byte(reg);
    ok &= i2c_write_byte(val);
    i2c_stop();
    return ok;
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t val = 0;
    i2c_start();
    if (!i2c_write_byte(0x0C)) { i2c_stop(); return 0; }
    if (!i2c_write_byte(reg))  { i2c_stop(); return 0; }
    i2c_start();                                /* 重复起始 */
    if (!i2c_write_byte(0x0D)) { i2c_stop(); return 0; }   /* 地址 + 读 */
    val = i2c_read_byte(0);                     /* NACK 收尾 */
    i2c_stop();
    return val;
}

/* =====================================================================
 * @brief  配置芯片 ABZ 分辨率（写 RAM，每次上电执行）
 * @param  ppr  线数（如 1024）
 * @retval 1=配置并读回校验通过；0=失败（I2C 无应答或读回不符）
 * ===================================================================== */
uint8_t MT6701_Abz_Configure(uint16_t ppr)
{
    uint16_t res = (uint16_t)(ppr - 1U);   /* 寄存器值 = 分辨率 - 1 */
    uint8_t  ok  = 0;

    /* 1. 切 I2C 模式：MODE 高 + Z 高 + PA0/PA1 开漏 */
    MT6701_SSI_MODE_PORT->BSRR = MT6701_SSI_MODE_PIN;   /* MODE 高 */
    pin_to_gpio_i2c();
    i2c_delay();

    /* 2. 写 ABZ_RES[9:8]（0x30，保留 UVW_RES 高 4 位）与 ABZ_RES[7:0]（0x31） */
    uint8_t reg30 = reg_read(0x30);
    reg30 = (reg30 & 0xF0U) | (uint8_t)((res >> 8) & 0x03U);
    if (reg_write(0x30, reg30) && reg_write(0x31, (uint8_t)(res & 0xFFU)))
    {
        /* 3. 读回校验 */
        ok = ((reg_read(0x30) & 0x03U) == (uint8_t)((res >> 8) & 0x03U)) &&
             (reg_read(0x31) == (uint8_t)(res & 0xFFU));
    }

    /* 4. 恢复引脚与 MODE（MODE 低 → 回 ABZ 模式） */
    pin_restore();
    MT6701_SSI_MODE_PORT->BSRR = (uint32_t)MT6701_SSI_MODE_PIN << 16U;

    return ok;
}
