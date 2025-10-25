/**
 ******************************************************************************
 * @file           : usart.c
 * @brief          : USART/UART driver implementation for STM32 with FreeRTOS
 * @details        : Thread-safe UART communication with printf support
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PRINTF_BUFFER_SIZE      256

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t usart_mutex = NULL;
static char printf_buffer[PRINTF_BUFFER_SIZE];

/* UART Handles - These will be initialized by STM32CubeMX */
UART_HandleTypeDef huart1;  // Communication UART
UART_HandleTypeDef huart2;  // Debug UART (for printf)

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef USART_WaitOnFlag(UART_HandleTypeDef *huart, uint32_t Flag,
                                          FlagStatus Status, uint32_t Tickstart,
                                          uint32_t Timeout);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize UART peripheral
 */
HAL_StatusTypeDef USART_Init(USART_Config_t *config)
{
    if (config == NULL || config->huart == NULL)
    {
        return HAL_ERROR;
    }

    /* Create mutex for thread-safe transmission */
    if (usart_mutex == NULL)
    {
        usart_mutex = xSemaphoreCreateMutex();
        if (usart_mutex == NULL)
        {
            return HAL_ERROR;
        }
    }

    /* Configure UART parameters */
    config->huart->Init.BaudRate = config->baudrate;
    config->huart->Init.WordLength = UART_WORDLENGTH_8B;
    config->huart->Init.StopBits = UART_STOPBITS_1;
    config->huart->Init.Parity = UART_PARITY_NONE;
    config->huart->Init.Mode = UART_MODE_TX_RX;
    config->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    config->huart->Init.OverSampling = UART_OVERSAMPLING_16;

    /* Initialize UART */
    if (HAL_UART_Init(config->huart) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  De-initialize UART peripheral
 */
HAL_StatusTypeDef USART_DeInit(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_UART_DeInit(huart);
}

/**
 * @brief  Transmit data via UART (blocking mode)
 */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                 uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status;

    /* Take mutex for thread-safe operation */
    if (usart_mutex != NULL)
    {
        if (xSemaphoreTake(usart_mutex, pdMS_TO_TICKS(Timeout)) != pdTRUE)
        {
            return HAL_TIMEOUT;
        }
    }

    /* Transmit data */
    status = HAL_UART_Transmit(huart, pData, Size, Timeout);

    /* Release mutex */
    if (usart_mutex != NULL)
    {
        xSemaphoreGive(usart_mutex);
    }

    return status;
}

/**
 * @brief  Receive data via UART (blocking mode)
 */
HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                                uint16_t Size, uint32_t Timeout)
{
    return HAL_UART_Receive(huart, pData, Size, Timeout);
}

/**
 * @brief  Transmit data via UART (interrupt mode)
 */
HAL_StatusTypeDef USART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size)
{
    return HAL_UART_Transmit_IT(huart, pData, Size);
}

/**
 * @brief  Receive data via UART (interrupt mode)
 */
HAL_StatusTypeDef USART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                   uint16_t Size)
{
    return HAL_UART_Receive_IT(huart, pData, Size);
}

/**
 * @brief  Transmit data via UART (DMA mode)
 */
HAL_StatusTypeDef USART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,
                                     uint16_t Size)
{
    return HAL_UART_Transmit_DMA(huart, pData, Size);
}

/**
 * @brief  Receive data via UART (DMA mode)
 */
HAL_StatusTypeDef USART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size)
{
    return HAL_UART_Receive_DMA(huart, pData, Size);
}

/**
 * @brief  Print formatted string via UART (printf-like)
 */
int USART_Printf(UART_HandleTypeDef *huart, const char *format, ...)
{
    va_list args;
    int len;

    /* Format string */
    va_start(args, format);
    len = vsnprintf(printf_buffer, PRINTF_BUFFER_SIZE, format, args);
    va_end(args);

    /* Check for errors */
    if (len < 0 || len >= PRINTF_BUFFER_SIZE)
    {
        return -1;
    }

    /* Transmit formatted string */
    if (USART_Transmit(huart, (uint8_t*)printf_buffer, len, USART_TIMEOUT_MS) != HAL_OK)
    {
        return -1;
    }

    return len;
}

/**
 * @brief  Send single character via UART (for printf redirection)
 * @note   Uses USART2 by default (configure in stm32_config.h)
 */
int __io_putchar(int ch)
{
    /* Wait until TXE flag is set */
    uint32_t timeout = HAL_GetTick() + USART_TIMEOUT_MS;
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == RESET)
    {
        if (HAL_GetTick() > timeout)
        {
            return -1;  // Timeout
        }
    }

    /* Transmit character */
    huart2.Instance->DR = (uint8_t)ch;

    return ch;
}

/**
 * @brief  Receive single character via UART (for scanf redirection)
 * @note   Uses USART2 by default (configure in stm32_config.h)
 */
int __io_getchar(void)
{
    uint8_t ch = 0;

    /* Wait until RXNE flag is set */
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET)
    {
        // Wait for data
    }

    /* Read character */
    ch = (uint8_t)(huart2.Instance->DR & 0xFF);

    return ch;
}

/**
 * @brief  Flush UART TX buffer
 */
void USART_Flush(UART_HandleTypeDef *huart)
{
    /* Wait until TC flag is set */
    uint32_t timeout = HAL_GetTick() + USART_TIMEOUT_MS;
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
        if (HAL_GetTick() > timeout)
        {
            break;  // Timeout
        }
    }
}

/**
 * @brief  Get number of bytes available in RX buffer
 * @note   This is a stub - implement with DMA or circular buffer for actual use
 */
uint16_t USART_Available(UART_HandleTypeDef *huart)
{
    // TODO: Implement with DMA or circular buffer
    return 0;
}

/* Weak callback implementations (can be overridden) -------------------------*/

/**
 * @brief  UART RX complete callback
 */
__weak void USART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE: This function should be overridden in user code */
}

/**
 * @brief  UART TX complete callback
 */
__weak void USART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE: This function should be overridden in user code */
}

/**
 * @brief  UART Error callback
 */
__weak void USART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);

    /* NOTE: This function should be overridden in user code */
}

/* HAL UART Callback overrides -----------------------------------------------*/

/**
 * @brief  RX Transfer completed callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    USART_RxCpltCallback(huart);
}

/**
 * @brief  TX Transfer completed callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    USART_TxCpltCallback(huart);
}

/**
 * @brief  UART error callback
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    USART_ErrorCallback(huart);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Wait for UART flag with timeout
 */
static HAL_StatusTypeDef USART_WaitOnFlag(UART_HandleTypeDef *huart, uint32_t Flag,
                                          FlagStatus Status, uint32_t Tickstart,
                                          uint32_t Timeout)
{
    /* Wait until flag is set or timeout */
    while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
    {
        /* Check for timeout */
        if (Timeout != HAL_MAX_DELAY)
        {
            if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
            {
                return HAL_TIMEOUT;
            }
        }
    }

    return HAL_OK;
}
