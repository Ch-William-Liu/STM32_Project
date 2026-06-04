/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    user_diskio.c
 * @brief   SPI microSD FatFs diskio driver
 ******************************************************************************
 */
/* USER CODE END Header */

#include <string.h>
#include <stdio.h>
#include <stdio.h>
#include "ff_gen_drv.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart3;
extern char uart_msg[128];

static volatile DSTATUS Stat = STA_NOINIT;

#define SD_SPI_HANDLE      hspi1
#define SD_CS_LOW()        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()       HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

#define CMD0               0
#define CMD1               1
#define CMD8               8
#define CMD9               9
#define CMD10              10
#define CMD12              12
#define CMD16              16
#define CMD17              17
#define CMD18              18
#define CMD23              23
#define CMD24              24
#define CMD25              25
#define CMD55              55
#define CMD58              58
#define ACMD41             41

#define CT_MMC             0x01
#define CT_SD1             0x02
#define CT_SD2             0x04
#define CT_BLOCK           0x08

static BYTE CardType = 0;

DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver =
{
    USER_initialize,
    USER_status,
    USER_read,
#if _USE_WRITE
    USER_write,
#endif
#if _USE_IOCTL == 1
    USER_ioctl,
#endif
};

static BYTE SPI_TxRx(BYTE data)
{
    BYTE rx;
    HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &data, &rx, 1, 100);
    return rx;
}

static void SPI_SendClock(void)
{
    for (int i = 0; i < 10; i++)
    {
        SPI_TxRx(0xFF);
    }
}

static int SD_WaitReady(UINT timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do
    {
        if (SPI_TxRx(0xFF) == 0xFF)
        {
            return 1;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return 0;
}

static void SD_Deselect(void)
{
    SD_CS_HIGH();
    SPI_TxRx(0xFF);
}

static int SD_Select(void)
{
    SD_CS_LOW();
    SPI_TxRx(0xFF);

    if (SD_WaitReady(500))
    {
        return 1;
    }

    SD_Deselect();
    return 0;
}

static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
    BYTE n, res;

    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        res = SD_SendCmd(CMD55, 0);

        if (res > 1)
        {
            return res;
        }
    }

    SD_Deselect();

    if (!SD_Select())
    {
        return 0xFF;
    }

    SPI_TxRx(0x40 | cmd);
    SPI_TxRx((BYTE)(arg >> 24));
    SPI_TxRx((BYTE)(arg >> 16));
    SPI_TxRx((BYTE)(arg >> 8));
    SPI_TxRx((BYTE)arg);

    n = 0x01;

    if (cmd == CMD0)
    {
        n = 0x95;
    }
    else if (cmd == CMD8)
    {
        n = 0x87;
    }

    SPI_TxRx(n);

    if (cmd == CMD12)
    {
        SPI_TxRx(0xFF);
    }

    n = 10;

    do
    {
        res = SPI_TxRx(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

static int SD_RxDataBlock(BYTE *buff, UINT btr)
{
    BYTE token;
    uint32_t start = HAL_GetTick();

    do
    {
        token = SPI_TxRx(0xFF);
    } while ((token == 0xFF) && ((HAL_GetTick() - start) < 200));

    if (token != 0xFE)
    {
        return 0;
    }

    for (UINT i = 0; i < btr; i++)
    {
        buff[i] = SPI_TxRx(0xFF);
    }

    SPI_TxRx(0xFF);
    SPI_TxRx(0xFF);

    return 1;
}

#if _USE_WRITE == 1
static int SD_TxDataBlock(const BYTE *buff, BYTE token)
{
    BYTE resp;

    if (!SD_WaitReady(500))
    {
        return 0;
    }

    SPI_TxRx(token);

    if (token != 0xFD)
    {
        for (UINT i = 0; i < 512; i++)
        {
            SPI_TxRx(buff[i]);
        }

        SPI_TxRx(0xFF);
        SPI_TxRx(0xFF);

        resp = SPI_TxRx(0xFF);

        if ((resp & 0x1F) != 0x05)
        {
            return 0;
        }
    }

    return 1;
}
#endif

DSTATUS USER_initialize(BYTE pdrv)
{
    BYTE n, cmd, ty, ocr[4];

    if (pdrv != 0)
    {
        return STA_NOINIT;
    }

    SD_CS_HIGH();
    HAL_Delay(100);

    SPI_SendClock();
    HAL_Delay(100);

    BYTE test;
    test = SPI_TxRx(0xFF);

    snprintf(uart_msg,
             128,
             "SPI TEST = 0x%02X\r\n",
             test);

    HAL_UART_Transmit(&huart3,
                      (uint8_t *)uart_msg,
                      strlen(uart_msg),
                      100);

    ty = 0;

    BYTE r1;
    r1 = SD_SendCmd(CMD0, 0);

    snprintf(uart_msg,
             128,
             "CMD0 = 0x%02X\r\n",
             r1);

    HAL_UART_Transmit(&huart3,
                      (uint8_t *)uart_msg,
                      strlen(uart_msg),
                      100);

    if (r1 == 1)
    {
        if (SD_SendCmd(CMD8, 0x1AA) == 1)
        {
            for (n = 0; n < 4; n++)
            {
                ocr[n] = SPI_TxRx(0xFF);
            }

            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                uint32_t start = HAL_GetTick();

                while ((HAL_GetTick() - start) < 1000)
                {
                    if (SD_SendCmd(0x80 | ACMD41, 1UL << 30) == 0)
                    {
                        break;
                    }
                }

                if ((HAL_GetTick() - start) < 1000 && SD_SendCmd(CMD58, 0) == 0)
                {
                    for (n = 0; n < 4; n++)
                    {
                        ocr[n] = SPI_TxRx(0xFF);
                    }

                    ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
                }
            }
        }
        else
        {
            if (SD_SendCmd(0x80 | ACMD41, 0) <= 1)
            {
                ty = CT_SD1;
                cmd = 0x80 | ACMD41;
            }
            else
            {
                ty = CT_MMC;
                cmd = CMD1;
            }

            uint32_t start = HAL_GetTick();

            while ((HAL_GetTick() - start) < 1000)
            {
                if (SD_SendCmd(cmd, 0) == 0)
                {
                    break;
                }
            }

            if (!((HAL_GetTick() - start) < 1000) || SD_SendCmd(CMD16, 512) != 0)
            {
                ty = 0;
            }
        }
    }

    CardType = ty;
    SD_Deselect();

    if (ty)
    {
        Stat &= ~STA_NOINIT;
    }
    else
    {
        Stat = STA_NOINIT;
    }

    return Stat;
}

DSTATUS USER_status(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }

    return Stat;
}

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count)
    {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (!(CardType & CT_BLOCK))
    {
        sector *= 512;
    }

    if (count == 1)
    {
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
        {
            count = 0;
        }
    }
    else
    {
        if (SD_SendCmd(CMD18, sector) == 0)
        {
            do
            {
                if (!SD_RxDataBlock(buff, 512))
                {
                    break;
                }

                buff += 512;
            } while (--count);

            SD_SendCmd(CMD12, 0);
        }
    }

    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count)
    {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if (!(CardType & CT_BLOCK))
    {
        sector *= 512;
    }

    if (count == 1)
    {
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
        {
            count = 0;
        }
    }
    else
    {
        if (CardType & CT_SD1 || CardType & CT_SD2)
        {
            SD_SendCmd(0x80 | CMD23, count);
        }

        if (SD_SendCmd(CMD25, sector) == 0)
        {
            do
            {
                if (!SD_TxDataBlock(buff, 0xFC))
                {
                    break;
                }

                buff += 512;
            } while (--count);

            if (!SD_TxDataBlock(0, 0xFD))
            {
                count = 1;
            }
        }
    }

    SD_Deselect();

    return count ? RES_ERROR : RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    DRESULT res;
    BYTE n, csd[16];
    DWORD csize;

    if (pdrv)
    {
        return RES_PARERR;
    }

    if (Stat & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    res = RES_ERROR;

    switch (cmd)
    {
        case CTRL_SYNC:
            if (SD_Select())
            {
                res = RES_OK;
            }
            break;

        case GET_SECTOR_COUNT:
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
            {
                if ((csd[0] >> 6) == 1)
                {
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD *)buff = csize << 10;
                }
                else
                {
                    n = (csd[5] & 15) +
                        ((csd[10] & 128) >> 7) +
                        ((csd[9] & 3) << 1) + 2;

                    csize = (csd[8] >> 6) +
                            ((WORD)csd[7] << 2) +
                            ((WORD)(csd[6] & 3) << 10) + 1;

                    *(DWORD *)buff = csize << (n - 9);
                }

                res = RES_OK;
            }
            break;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            res = RES_OK;
            break;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
            break;
    }

    SD_Deselect();

    return res;
}
#endif