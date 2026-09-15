#include "encoder.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi3;

#define SSI_CSN_PORT    GPIOA
#define SSI_CSN_PIN     GPIO_PIN_15

#define SSI_MODE_PORT   GPIOA
#define SSI_MODE_PIN    GPIO_PIN_2

#define SSI_CSN_LOW()   HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_RESET)
#define SSI_CSN_HIGH()  HAL_GPIO_WritePin(SSI_CSN_PORT, SSI_CSN_PIN, GPIO_PIN_SET)

static uint8_t rx_buf[3];

/* DMA 读取状态 */
static volatile uint16_t enc_angle = 0;   /* 最近一次 CRC 校验通过的角度 */
static volatile uint8_t  enc_valid = 0;   /* 是否已经读到过有效帧 */
static volatile uint8_t  enc_busy  = 0;   /* DMA 是否正在搬运 */


void Encoder_HW_Init(void)
{
    // 选择模式的引脚。使用ssi：mode=1；CSN=0传输开始；CLK下降沿传输数据
    HAL_GPIO_WritePin(SSI_MODE_PORT, SSI_MODE_PIN, GPIO_PIN_SET);//mode
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


uint8_t Encoder_Process_RawData(Encoder_RawData_t *data)
{
    // 1. 先把下一帧发出去（非阻塞，立即返回）
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
