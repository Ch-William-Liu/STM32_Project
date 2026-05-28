#include "sd_spi.h"
#include <stdio.h>
#include "ff_gen_drv.h"

extern SPI_HandleTypeDef hspi1;

#define SD_CS_GPIO_PORT GPIOB
#define SD_CS_PIN       GPIO_PIN_6

#define SD_SPI_TIMEOUT  1000

static DSTATUS Stat = STA_NOINIT;

static void SD_Select(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

static void SD_Deselect(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, SD_SPI_TIMEOUT);
    return rx;
}

static void SPI_SendDummyClocks(void)
{
    SD_Deselect();

    for (int i = 0; i < 10; i++)
    {
        SPI_TxRx(0xFF);
    }
}

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t res;

    SD_Deselect();
    SPI_TxRx(0xFF);
    SD_Select();
    SPI_TxRx(0xFF);

    SPI_TxRx(cmd | 0x40);
    SPI_TxRx((uint8_t)(arg >> 24));
    SPI_TxRx((uint8_t)(arg >> 16));
    SPI_TxRx((uint8_t)(arg >> 8));
    SPI_TxRx((uint8_t)arg);
    SPI_TxRx(crc);

    for (int i = 0; i < 10; i++)
    {
        res = SPI_TxRx(0xFF);
        if ((res & 0x80) == 0)
        {
            return res;
        }
    }

    return 0xFF;
}

DSTATUS SD_SPI_initialize(BYTE pdrv)
{
    uint8_t res;
    uint16_t retry = 0;

    printf("SD init start\r\n");
    (void)pdrv;

    SPI_SendDummyClocks();

    res = SD_SendCmd(0, 0, 0x95);
    printf("CMD0 res = 0x%02X\r\n", res);
    if (res != 0x01)
    {
        Stat = STA_NOINIT;
        SD_Deselect();
        return Stat;
    }

    uint8_t ocr[4];

    res = SD_SendCmd(8, 0x000001AA, 0x87);
    printf("CMD8 res = 0x%02X\r\n", res);

    if (res == 0x01)
    {
        for (int i = 0; i < 4; i++)
        {
            ocr[i] = SPI_TxRx(0xFF);
        }

        printf("CMD8 OCR = %02X %02X %02X %02X\r\n",
            ocr[0], ocr[1], ocr[2], ocr[3]);

        retry = 0;

        do
        {
            SD_SendCmd(55, 0, 0xFF);
            res = SD_SendCmd(41, 0x40000000, 0xFF);
            retry++;
        }
        while (res != 0x00 && retry < 10000);

        printf("ACMD41 res = 0x%02X retry=%u\r\n", res, retry);

        if (res != 0x00)
        {
            Stat = STA_NOINIT;
            SD_Deselect();
            return Stat;
        }

        res = SD_SendCmd(58, 0, 0xFF);
        printf("CMD58 res = 0x%02X\r\n", res);

        if (res == 0x00)
        {
            for (int i = 0; i < 4; i++)
            {
                ocr[i] = SPI_TxRx(0xFF);
            }

            printf("CMD58 OCR = %02X %02X %02X %02X\r\n",
                ocr[0], ocr[1], ocr[2], ocr[3]);
        }

        Stat &= ~STA_NOINIT;
        printf("Stat after init = %02X\r\n", Stat);
    }
    else
    {
        Stat = STA_NOINIT;
    }

    SD_Deselect();
    SPI_TxRx(0xFF);

    if (res == 0x00)
    {
        Stat &= ~STA_NOINIT;
    }
    else
    {
        Stat = STA_NOINIT;
    }

    return Stat;
}

DSTATUS SD_SPI_status(BYTE pdrv)
{
    (void)pdrv;
    printf("SD_SPI_status Stat = %02X\r\n", Stat);
    return Stat;
}

static int SD_WaitReady(void)
{
    uint8_t res;
    uint32_t timeout = 50000;

    do
    {
        res = SPI_TxRx(0xFF);
        if (res == 0xFF)
        {
            return 1;
        }
    }
    while (--timeout);

    return 0;
}

static int SD_ReadBlock(uint8_t *buff, uint32_t sector)
{
    uint8_t token;
    uint32_t timeout = 50000;

    if (SD_SendCmd(17, sector, 0xFF) != 0x00)
    {
        SD_Deselect();
        return 0;
    }

    do
    {
        token = SPI_TxRx(0xFF);
    }
    while (token == 0xFF && --timeout);

    if (token != 0xFE)
    {
        SD_Deselect();
        return 0;
    }

    for (uint32_t i = 0; i < 512; i++)
    {
        buff[i] = SPI_TxRx(0xFF);
    }

    SPI_TxRx(0xFF);
    SPI_TxRx(0xFF);

    SD_Deselect();
    SPI_TxRx(0xFF);

    return 1;
}

DRESULT SD_SPI_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;

    if (!count)
    {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    for (UINT i = 0; i < count; i++)
    {
        if (!SD_ReadBlock(buff + i * 512, sector + i))
        {
            return RES_ERROR;
        }
    }

    return RES_OK;
}

static int SD_WriteBlock(const uint8_t *buff, uint32_t sector)
{
    if (SD_SendCmd(24, sector, 0xFF) != 0x00)
    {
        SD_Deselect();
        return 0;
    }

    SPI_TxRx(0xFF);
    SPI_TxRx(0xFE);

    for (uint32_t i = 0; i < 512; i++)
    {
        SPI_TxRx(buff[i]);
    }

    SPI_TxRx(0xFF);
    SPI_TxRx(0xFF);

    uint8_t resp = SPI_TxRx(0xFF);

    if ((resp & 0x1F) != 0x05)
    {
        SD_Deselect();
        return 0;
    }

    if (!SD_WaitReady())
    {
        SD_Deselect();
        return 0;
    }

    SD_Deselect();
    SPI_TxRx(0xFF);

    return 1;
}

DRESULT SD_SPI_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;

    if (!count)
    {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    for (UINT i = 0; i < count; i++)
    {
        if (!SD_WriteBlock(buff + i * 512, sector + i))
        {
            return RES_ERROR;
        }
    }

    return RES_OK;
}

DRESULT SD_SPI_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
        case CTRL_SYNC:
            SD_Select();
            if (SD_WaitReady())
            {
                SD_Deselect();
                return RES_OK;
            }
            SD_Deselect();
            return RES_ERROR;

        case GET_SECTOR_COUNT:
            *(DWORD *)buff = 31291392;
            return RES_OK;
        
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}