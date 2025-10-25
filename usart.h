/**
 ******************************************************************************
 * @file           : usart.h
 * @brief          : USART/UART driver header for STM32 with FreeRTOS
 * @details        : Provides UART communication for debugging and data exchange
 ******************************************************************************
 * @attention
 *
 * Features:
 * - Printf redirection support
 * - DMA and interrupt-based transmission
 * - Thread-safe with FreeRTOS mutexes
 * - Configurable baudrate and buffer sizes
 *
 * Compatible with:
 * - STM32F4xx, STM32F7xx, STM32H7xx series
 * - STM32CubeIDE HAL drivers
 * - FreeRTOS
 *
 ******************************************************************************
 */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  // Altere para seu MCU: stm32f7xx_hal.h, stm32h7xx_hal.h, etc.
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART Configuration Structure
 */
typedef struct {
    UART_HandleTypeDef *huart;      /**< HAL UART handle */
    uint32_t baudrate;              /**< Baudrate (9600, 115200, etc.) */
    uint16_t tx_buffer_size;        /**< TX buffer size in bytes */
    uint16_t rx_buffer_size;        /**< RX buffer size in bytes */
    bool use_dma;                   /**< Enable DMA for TX/RX */
    SemaphoreHandle_t tx_mutex;     /**< Mutex for thread-safe transmission */
} USART_Config_t;

/* Exported constants --------------------------------------------------------*/

/* Default UART settings */
#define USART_DEFAULT_BAUDRATE          115200
#define USART_TX_BUFFER_SIZE            512
#define USART_RX_BUFFER_SIZE            512
#define USART_TIMEOUT_MS                1000

/* UART instances - Configure based on your hardware */
#define USART_DEBUG                     USART2  /**< Debug/printf UART */
#define USART_COMMUNICATION             USART1  /**< Inter-node communication */

/* Exported macro ------------------------------------------------------------*/

/**
 * @brief Check if UART is ready for transmission
 */
#define USART_IS_TX_READY(huart)        (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET)

/**
 * @brief Check if UART has received data
 */
#define USART_IS_RX_READY(huart)        (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize UART peripheral
 * @param  config: Pointer to UART configuration structure
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Init(USART_Config_t *config);

/**
 * @brief  De-initialize UART peripheral
 * @param  huart: Pointer to UART handle
 * @retval HAL status
 */
HAL_StatusTypeDef USART_DeInit(UART_HandleTypeDef *huart);

/**
 * @brief  Transmit data via UART (blocking mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to send
 * @param  Timeout: Timeout in milliseconds
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                 uint16_t Size, uint32_t Timeout);

/**
 * @brief  Receive data via UART (blocking mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to receive
 * @param  Timeout: Timeout in milliseconds
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
                                uint16_t Size, uint32_t Timeout);

/**
 * @brief  Transmit data via UART (interrupt mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to send
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size);

/**
 * @brief  Receive data via UART (interrupt mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to receive
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData,
                                   uint16_t Size);

/**
 * @brief  Transmit data via UART (DMA mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to send
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData,
                                     uint16_t Size);

/**
 * @brief  Receive data via UART (DMA mode)
 * @param  huart: Pointer to UART handle
 * @param  pData: Pointer to data buffer
 * @param  Size: Number of bytes to receive
 * @retval HAL status
 */
HAL_StatusTypeDef USART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData,
                                    uint16_t Size);

/**
 * @brief  Print formatted string via UART (printf-like)
 * @param  huart: Pointer to UART handle
 * @param  format: Format string (printf syntax)
 * @param  ...: Variable arguments
 * @retval Number of characters printed
 */
int USART_Printf(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @brief  Send single character via UART (for printf redirection)
 * @param  ch: Character to send
 * @retval Character sent, or -1 on error
 *
 * @note   This function is used by syscalls.c for printf redirection
 */
int __io_putchar(int ch);

/**
 * @brief  Receive single character via UART (for scanf redirection)
 * @retval Character received
 *
 * @note   This function is used by syscalls.c for scanf redirection
 */
int __io_getchar(void);

/**
 * @brief  Flush UART TX buffer
 * @param  huart: Pointer to UART handle
 * @retval None
 */
void USART_Flush(UART_HandleTypeDef *huart);

/**
 * @brief  Get number of bytes available in RX buffer
 * @param  huart: Pointer to UART handle
 * @retval Number of bytes available
 */
uint16_t USART_Available(UART_HandleTypeDef *huart);

/**
 * @brief  UART RX complete callback (override this in your code)
 * @param  huart: Pointer to UART handle
 * @retval None
 */
void USART_RxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief  UART TX complete callback (override this in your code)
 * @param  huart: Pointer to UART handle
 * @retval None
 */
void USART_TxCpltCallback(UART_HandleTypeDef *huart);

/**
 * @brief  UART Error callback (override this in your code)
 * @param  huart: Pointer to UART handle
 * @retval None
 */
void USART_ErrorCallback(UART_HandleTypeDef *huart);

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;  /**< UART1 handle (communication) */
extern UART_HandleTypeDef huart2;  /**< UART2 handle (debug/printf) */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
