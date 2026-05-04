#include "sd_spi.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define SD_CS_LOW()     HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

static uint8_t spi_txrx(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);

    return rx;
} // end function spi_txrx

static void spi_dummy_clocks(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        spi_txrx(0xFF);
    } // end for
} // end function spi_dummy_clocks

static uint8_t sd_wait_ready(void)
{
    uint32_t timeout = 500000;

    while (timeout--)
    {
        if (spi_txrx(0xFF) == 0xFF)
        {
            return 1;
        } // end if 
    } // end while

    return 0;
} // end function ad_wait_clocks

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t response;
    uint8_t packet[6];

    sd_wait_ready();

    packet[0] = 0x40 | cmd;
    packet[1] = (uint8_t)(arg >> 24);
    packet[2] = (uint8_t)(arg >> 16);
    packet[3] = (uint8_t)(arg >> 8);
    packet[4] = (uint8_t)(arg);
    packet[5] = crc;

    for (uint8_t i = 0; i < 6; i++)
    {
        spi_txrx(packet[i]);
    } // end for

    for (uint8_t i = 0; i < 10; i++)
    {
        response = spi_txrx(0xFF);

        if (!(response & 0x80))
        {
            return response;
        } // end if
        
    } // end for
    
    return 0xFF;

} // end function sd_send_cmd

uint8_t SD_SPI_Init(void)
{
    uint8_t response;
    uint32_t timeout;

    SD_CS_HIGH();
    spi_dummy_clocks(10);

    SD_CS_LOW();

    response = sd_send_cmd(0 , 0 , 0x95);

    if (response != 0x01)
    {
        SD_CS_HIGH();
        return 0;
    } // end if 

    response = sd_send_cmd(8 , 0x000001AA , 0x87);

    for (int i = 0; i < 4; i++)
    {
        spi_txrx(0xFF);
    } // end for
    
    timeout = 100000;

    do
    {
        sd_send_cmd(55 , 0 , 0x01);
        response = sd_send_cmd(41, 0x40000000, 0x01);
    } while (response != 0x00 && --timeout);
    
    SD_CS_HIGH();
    spi_txrx(0xFF);

    if (timeout == 0)
    {
        return 0;
    } // end if 

    return 1;
} // end function SD_SPI_Init

uint8_t SD_SPI_ReadBlocks(uint8_t *buf, uint32_t sector, uint32_t count)
{
    for (uint32_t c = 0; c < count; c++)
    {
        uint8_t token;
        uint32_t timeout;

        SD_CS_LOW();

        if (sd_send_cmd(17, sector + c, 0x01) != 0x00)
        {
            SD_CS_HIGH();
            return 0;
        }

        timeout = 1000000;

        do
        {
            token = spi_txrx(0xFF);
        } while (token == 0xFF && --timeout);

        if (token != 0xFE)
        {
            SD_CS_HIGH();
            return 0;
        }

        for (uint32_t i = 0; i < 512; i++)
        {
            buf[c * 512 + i] = spi_txrx(0xFF);
        }

        spi_txrx(0xFF);
        spi_txrx(0xFF);

        SD_CS_HIGH();
        spi_txrx(0xFF);
    }

    return 1;
}

uint8_t SD_SPI_WriteBlocks(const uint8_t *buf, uint32_t sector, uint32_t count)
{
    for (uint32_t c = 0; c < count; c++)
    {
        uint8_t response;

        SD_CS_LOW();

        if (sd_send_cmd(24, sector + c, 0x01) != 0x00)
        {
            SD_CS_HIGH();
            return 0;
        }

        spi_txrx(0xFF);
        spi_txrx(0xFE);

        for (uint32_t i = 0; i < 512; i++)
        {
            spi_txrx(buf[c * 512 + i]);
        }

        spi_txrx(0xFF);
        spi_txrx(0xFF);

        response = spi_txrx(0xFF);

        if ((response & 0x1F) != 0x05)
        {
            SD_CS_HIGH();
            return 0;
        }

        if (!sd_wait_ready())
        {
            SD_CS_HIGH();
            return 0;
        }

        SD_CS_HIGH();
        spi_txrx(0xFF);
    }

    return 1;
}