#include "uart_command.h"
#include "ds3231.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define UART_COMMAND_BUFFER_SIZE      100U
#define UART_TX_BUFFER_SIZE           160U
#define UART_TX_TIMEOUT_MS            1000U

static UART_HandleTypeDef *commandUart = NULL;
static I2C_HandleTypeDef *rtcI2c = NULL;

static uint8_t receivedByte;

static char receiveBuffer[UART_COMMAND_BUFFER_SIZE];
static volatile uint16_t receiveIndex = 0U;

static char commandBuffer[UART_COMMAND_BUFFER_SIZE];
static volatile uint8_t commandReady = 0U;
static volatile uint8_t bufferOverflow = 0U;

static void UART_Command_SendText(const char *text)
{
  if ((commandUart == NULL) || (text == NULL))
  {
    return;
  } // end if uart null and text null

  HAL_UART_Transmit(commandUart , (uint8_t *)text , (uint16_t)strlen(text) , UART_TX_TIMEOUT_MS);
} // end function UART_Command_SendText

static void UART_Command_SendDateTime(const char *prefix, const DS3231_DateTime_t *dateTime)
{
  char transmitBuffer[UART_TX_BUFFER_SIZE];

  if ((prefix == NULL) || (dateTime == NULL))
  {
    return;
  } // end if prefix or dateTime is NULL

  snprintf(transmitBuffer , sizeof(transmitBuffer) , "%s,%04u-%02u-%02u %02u:%02u:%02u\r\n", 
          prefix, dateTime->year , dateTime->month , dateTime->date , dateTime->hour , dateTime->minute , dateTime->second);

  UART_Command_SendText(transmitBuffer);
} // end function UART_Command_SendDateTime

static void UART_Command_HandleSetTime(const char *command)
{
  int year , month , date , hour , minute , second;

  int parsedCount;
  DS3231_DateTime_t requestTime;
  DS3231_DateTime_t readBackTime;
  HAL_StatusTypeDef status;

  parsedCount = sscanf(command , "SETTIME,%d,%d,%d,%d,%d,%d" , &year , &month , &date , &hour , &minute , &second);

  if (parsedCount != 6)
  {
    UART_Command_SendText("ERROR,FORMAT\r\n");
    return;
  } // end if parsedCount != 6

  requestTime.year = (uint16_t)year;
  requestTime.month = (uint8_t)month;
  requestTime.date = (uint8_t)date;
  requestTime.hour = (uint8_t)hour;
  requestTime.minute = (uint8_t)minute;
  requestTime.second = (uint8_t)second;

  requestTime.dayOfWeek = DS3231_CalculateDayOfWeek(requestTime.year , requestTime.month , requestTime.date);

  if (DS3231_IsDateTimeValid(&requestTime))
  {
    UART_Command_SendText("ERROR,DATETIME\r\n");
    return;
  } // end if requestTime is invalid

  status = DS3231_SetDateTime(rtcI2c , &requestTime);

  if (status != HAL_OK)
  {
    UART_Command_SendText("ERROR,RTC_WRITE\r\n");
    return;
  } // end if settime not ok

  HAL_Delay(10U);

  status = DS3231_GetDateTime(rtcI2c , &readBackTime);

  if (status != HAL_OK)
  {
    UART_Command_SendText("ERROR,RTC_READBACK\r\n");
    return;
  } // end if not able to readback

  UART_Command_SendDateTime("OK",&readBackTime);
} // end function UART_Command_HandleSetTime

static void UART_Command_HandleGetTime(void)
{
  DS3231_DateTime_t dateTime;
  HAL_StatusTypeDef status;

  status = DS3231_GetDateTime(rtcI2c , &dateTime);

  if (status != HAL_OK)
  {
    UART_Command_SendText("ERROR,RTC_READ\r\n");
    return;
  } // end if unable get time

  UART_Command_SendDateTime("TIME",&dateTime);
} // end function UART_Command_HandleGetTime

static void UART_Command_HandlePing(void)
{
  UART_Command_SendText("PONG\r\n");
} // end function UART_Command_HandlePing

static void UART_Command_HandleHelp(void)
{
  UART_Command_SendText(
        "COMMANDS:\r\n"
        "SETTIME,YYYY,MM,DD,HH,MM,SS\r\n"
        "GETTIME\r\n"
        "PING\r\n"
        "HELP\r\n");
} // end function UART_Command_HandleHelp

static void UART_Command_HandleCommand(const char *command)
{
  if (command == NULL)
  {
    return;
  } // end if not command

  if (strncmp(command , "SETTIME," , 8U) == 0)
  {
    UART_Command_HandleSetTime(command);
  } // end if command=SETTIME
  else if (strcmp(command , "GETTIME") == 0)
  {
    UART_Command_HandleGetTime();
  } // end if command=GETTIME
  else if (strcmp(command , "PING") == 0)
  {
    UART_Command_HandlePing();
  } // end if command=PING
  else if (strcmp(command , "HELP") == 0)
  {
    UART_Command_HandleHelp();
  }  // end if command=HELP
  else
  {
    UART_Command_SendText("ERROR,UNKNOWN_COMMAND,USE HELP\r\n");
  } // end if unknown command
} // end function UART_Command_HandleCommand

HAL_StatusTypeDef UART_Command_Init(UART_HandleTypeDef *huart, I2C_HandleTypeDef *hi2c)
{
  if ((huart == NULL) || (hi2c == NULL))
  {
    return HAL_ERROR;
  } // end if huart or i2c is NULL

  commandUart = huart;
  rtcI2c = hi2c;

  receiveIndex = 0U;
  commandReady = 0U;
  bufferOverflow = 0U;

  memset(receiveBuffer , 0 , sizeof(receiveBuffer));
  memset(commandBuffer , 0 , sizeof(commandBuffer));

  return HAL_UART_Receive_IT(commandUart , &receivedByte , 1U);
} // end function UART_Command_Init

void UART_Command_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((commandUart = NULL) || (huart != commandUart))
  {
    return;
  } // end if

  if (receivedByte == '\n')
  {
    if (!commandReady)
    {
      receiveBuffer[receiveIndex] = '\0';

      memcpy(commandBuffer , receiveBuffer , receiveIndex + 1U);
    } // end if not ready for command

    receiveIndex = 0U;
    memset(receiveBuffer , 0 , sizeof(receiveBuffer));
  } // end if line end
  else if (receivedByte == '\r')
  {
    /* ignore */
  } // end if line end
  else
  {
    if (receiveIndex < UART_COMMAND_BUFFER_SIZE - 1U)
    {
      receiveBuffer[receiveIndex] = (char)receivedByte;

      receiveIndex++;
    } // end if index smaller than buffer size
    else
    {
      receiveIndex = 0U;
      bufferOverflow = 1U;

      memset(receiveBuffer , 0 , sizeof(receiveBuffer));
    } // end else
  } // end else

  HAL_UART_Receive_IT(commandUart , &receivedByte , 1U);
} // end function UART_Command_RxCpltCallback

void UART_Command_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((commandUart == NULL) || (huart != commandUart))
  {
    return;
  } // end if uart and not ok

  /* reset the status once error occur*/
  receiveIndex = 0U;

  memset(receiveBuffer , 0 , sizeof(receiveBuffer));

  HAL_UART_Receive_IT(commandUart , &receivedByte , 1U);
} // end function UART_Command_ErrorCallback 

void UART_Command_Process(void)
{
  char localCommand[UART_COMMAND_BUFFER_SIZE];

  if (bufferOverflow)
  {
    bufferOverflow = 0U;

    UART_Command_SendText("ERROR,BUFFER_OVERFLOW\r\n");
  } // end if buffer overflow

  if (!commandReady)
  {
    return;
  } // end if command not ready

  __disable_irq();
  /* close ineterrupt temporary, copy command to local command */
  strncpy(localCommand , commandBuffer , sizeof(localCommand) - 1U);

  localCommand[sizeof(localCommand) - 1U] = '\0';

  commandReady = 0U;

  __enable_irq();

  UART_Command_HandleCommand(localCommand);

} // end UART_Command_Process