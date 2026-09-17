#include "mt6701.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi3;

/* 寄存器级 GPIO 操作：BSRR 原子置位/复位，省去 HAL 函数调用
   低 16 位写 1 = 置高，高 16 位写 1 = 置低 */
#define SSI_CSN_LOW()    (MT6701_SSI_CSN_PORT->BSRR = (uint32_t)MT6701_SSI_CSN_PIN << 16U)
#define SSI_CSN_HIGH()   (MT6701_SSI_CSN_PORT->BSRR = (uint32_t)MT6701_SSI_CSN_PIN)

static uint8_t rx_buf[3];

/* DMA 读取状态 */
static volatile uint16_t enc_angle = 0;   /* 最近一次 CRC 校验通过的角度 */
static volatile uint8_t  enc_valid = 0;   /* 是否已经读到过有效帧 */
static volatile uint8_t  enc_busy  = 0;   /* DMA 是否正在搬运 */


void MT6701_Ssi_Init(void)
{
    /* MODE 置高选择 SSI 模式（CSN 由软件控制，先拉高保持空闲） */
    MT6701_SSI_MODE_PORT->BSRR = (uint32_t)MT6701_SSI_MODE_PIN;
    SSI_CSN_HIGH();

    enc_angle = 0;
    enc_valid = 0;
    enc_busy  = 0;
}


static uint8_t CalcCRC(uint32_t data18)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < 18; i++)
    {
        uint8_t bit = (data18 >> (17 - i)) & 0x01;
        uint8_t msb = (crc >> 5) & 0x01;
        crc <<= 1;
        if (msb ^ bit)
            crc ^= 0x03;
        crc &= 0x3F;
    }
    return crc;
}


/* DMA 搬完 3 字节后由 DMA1_Channel2 中断进入。
   HAL 在回调前已关闭 SPI 并清空 FIFO，所以这里拉高 CSN 是安全的 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI3) return;

    SSI_CSN_HIGH();     // 一帧结束，释放片选
    enc_busy = 0;

    uint32_t raw24 = ((uint32_t)rx_buf[0] << 16) |
                     ((uint32_t)rx_buf[1] << 8)  |
                     (uint32_t)rx_buf[2];

    if ((raw24 & 0x3F) == CalcCRC((raw24 >> 6) & 0x3FFFF))
    {
        enc_angle = (raw24 >> 10) & 0x3FFF;     // 校验通过才更新
        enc_valid = 1;
    }
}


uint8_t MT6701_Ssi_Process_RawData(Encoder_RawData_t *data)
{
    // 1. 先把下一帧发出去（非阻塞，立即返回；CSN 提前拉低保证建立时间 TL>=100ns）
    if (!enc_busy)
    {
        SSI_CSN_LOW();
        enc_busy = 1;
        if (HAL_SPI_Receive_DMA(&hspi3, rx_buf, 3) != HAL_OK)
        {
            enc_busy = 0;
            SSI_CSN_HIGH();     // 启动失败，别把 CSN 留在低电平
        }
    }

    // 2. 返回上一帧的结果（第一次调用时还没有数据，返回 0）
    if (!enc_valid) return 0;

    data->angle = enc_angle;
    return 1;
}
