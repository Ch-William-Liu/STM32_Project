#include "fatfs_sd.h"

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

static uint8_t spi_txrx(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void sd_spi_dummy_clocks(void)
{
    SD_CS_HIGH();

    for (int i = 0; i < 10; i++)
    {
        spi_txrx(0xFF);
    }
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t res;

    SD_CS_LOW();

    spi_txrx(0x40 | cmd);
    spi_txrx((arg >> 24) & 0xFF);
    spi_txrx((arg >> 16) & 0xFF);
    spi_txrx((arg >> 8) & 0xFF);
    spi_txrx(arg & 0xFF);
    spi_txrx(crc);

    spi_txrx(0xFF);     // important dummy clock

    for (int i = 0; i < 10; i++)
    {
        res = spi_txrx(0xFF);
        if ((res & 0x80) == 0)
        {
            return res;
        }
    }

    return 0xFF;
}

uint8_t SD_SPI_Init(void)
{
    uint8_t res;
    uint32_t timeout;

    SD_CS_HIGH();
    HAL_Delay(100);

    sd_spi_dummy_clocks();

    res = sd_send_cmd(0, 0, 0x95);
    SD_CS_HIGH();
    spi_txrx(0xFF);

    printf("SD CMD0 res = 0x%02X\r\n", res);

    if (res != 0x01)
    {
        return 1;
    }

    res = sd_send_cmd(8, 0x000001AA, 0x87);
    printf("SD CMD8 res = 0x%02X\r\n", res);

    for (int i = 0; i < 4; i++)
    {
        uint8_t r7 = spi_txrx(0xFF);
        printf("CMD8 R7[%d]=0x%02X\r\n", i, r7);
    }

    SD_CS_HIGH();
    spi_txrx(0xFF);

    timeout = HAL_GetTick();

    do
    {
        res = sd_send_cmd(55, 0, 0x01);
        SD_CS_HIGH();
        spi_txrx(0xFF);

        printf("SD CMD55 res = 0x%02X\r\n", res);

        res = sd_send_cmd(41, 0x40000000, 0x01);
        SD_CS_HIGH();
        spi_txrx(0xFF);

        printf("SD ACMD41 res = 0x%02X\r\n", res);

        if ((HAL_GetTick() - timeout) > 5000)
        {
            printf("SD ACMD41 timeout\r\n");
            return 2;
        }

        HAL_Delay(10);

    } while (res != 0x00);

    printf("SD SPI init OK\r\n");

    return 0;
}

uint8_t SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count)
{
    if (count != 1)
    {
        return 1;
    }

    uint8_t res = sd_send_cmd(17, sector, 0x01);

    if (res != 0x00)
    {
        SD_CS_HIGH();
        return 2;
    }

    uint32_t timeout = HAL_GetTick();

    while (spi_txrx(0xFF) != 0xFE)
    {
        if ((HAL_GetTick() - timeout) > 1000)
        {
            SD_CS_HIGH();
            return 3;
        }
    }

    for (uint16_t i = 0; i < 512; i++)
    {
        buff[i] = spi_txrx(0xFF);
    }

    spi_txrx(0xFF);
    spi_txrx(0xFF);

    SD_CS_HIGH();
    spi_txrx(0xFF);

    return 0;
}

uint8_t SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count)
{
    if (count != 1)
    {
        return 1;
    }

    uint8_t res = sd_send_cmd(24, sector, 0x01);

    if (res != 0x00)
    {
        SD_CS_HIGH();
        return 2;
    }

    spi_txrx(0xFF);
    spi_txrx(0xFE);

    for (uint16_t i = 0; i < 512; i++)
    {
        spi_txrx(buff[i]);
    }

    spi_txrx(0xFF);
    spi_txrx(0xFF);

    res = spi_txrx(0xFF);

    if ((res & 0x1F) != 0x05)
    {
        SD_CS_HIGH();
        return 3;
    }

    uint32_t timeout = HAL_GetTick();

    while (spi_txrx(0xFF) == 0x00)
    {
        if ((HAL_GetTick() - timeout) > 1000)
        {
            SD_CS_HIGH();
            return 4;
        }
    }

    SD_CS_HIGH();
    spi_txrx(0xFF);

    return 0;
}

uint8_t SD_SPI_GetSectorCount(uint32_t *sector_count)
{
    *sector_count = 32768;  // temporary value, enough for FatFs basic operation
    return 0;
}