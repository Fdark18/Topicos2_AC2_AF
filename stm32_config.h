/**
 ******************************************************************************
 * @file           : stm32_config.h
 * @brief          : STM32 configuration for FreeRTOS conveyor belt system
 * @details        : Hardware configuration, pinout definitions, and system parameters
 ******************************************************************************
 * @attention
 *
 * This file centralizes all STM32-specific configurations for the conveyor
 * belt system. Modify this file to match your hardware setup.
 *
 * Supported MCUs:
 * - STM32F4xx series (default: STM32F407VG)
 * - STM32F7xx series
 * - STM32H7xx series
 *
 * Hardware Requirements:
 * - Emergency button (digital input)
 * - DC motor with PWM control
 * - 2x solenoid pistons (digital outputs)
 * - Status LEDs
 * - UART for debugging (115200 baud)
 *
 ******************************************************************************
 */

#ifndef __STM32_CONFIG_H
#define __STM32_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  // Change to: stm32f7xx_hal.h, stm32h7xx_hal.h, etc.

/* MCU Configuration ---------------------------------------------------------*/

/**
 * @brief MCU Family Selection
 * Uncomment the appropriate line for your MCU
 */
// #define STM32F401xE
// #define STM32F407xx
#define STM32F407xx     // Default: STM32F407VG Discovery Board
// #define STM32F429xx
// #define STM32F746xx
// #define STM32H743xx

/**
 * @brief System Clock Configuration
 */
#define SYSTEM_CLOCK_MHZ            168     /**< System clock in MHz (STM32F407) */
#define APB1_TIMER_CLOCK_MHZ        84      /**< APB1 timer clock (for TIM2-7) */
#define APB2_TIMER_CLOCK_MHZ        168     /**< APB2 timer clock (for TIM1, TIM8-11) */

/* FreeRTOS Configuration ----------------------------------------------------*/

/**
 * @brief FreeRTOS Heap Size
 * Adjust based on your RAM availability
 */
#define configTOTAL_HEAP_SIZE       ((size_t)(40 * 1024))  /**< 40KB heap */

/**
 * @brief FreeRTOS Tick Rate
 */
#define configTICK_RATE_HZ          1000    /**< 1ms tick period */

/**
 * @brief Stack sizes for each task (in words, not bytes)
 * STM32 uses 32-bit words, so 128 words = 512 bytes
 */
#define SAFETY_TASK_STACK_SIZE      256     /**< Safety node stack (1KB) */
#define CAMERA_TASK_STACK_SIZE      512     /**< Camera node stack (2KB) */
#define PISTON_TASK_STACK_SIZE      256     /**< Piston node stack (1KB) */
#define CONVEYOR_TASK_STACK_SIZE    256     /**< Conveyor node stack (1KB) */
#define MONITOR_TASK_STACK_SIZE     512     /**< Monitor task stack (2KB) */

/* GPIO Pin Definitions ------------------------------------------------------*/

/**
 * @brief Emergency Stop Button
 * Active LOW with internal pull-up
 */
#define EMERGENCY_BTN_PORT          GPIOA
#define EMERGENCY_BTN_PIN           GPIO_PIN_0
#define EMERGENCY_BTN_GPIO_CLK()    __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Conveyor Belt Motor (PWM Control)
 * TIM4 Channel 1 (PB6)
 */
#define MOTOR_PWM_PORT              GPIOB
#define MOTOR_PWM_PIN               GPIO_PIN_6
#define MOTOR_PWM_GPIO_CLK()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTOR_PWM_TIMER             TIM4
#define MOTOR_PWM_CHANNEL           TIM_CHANNEL_1
#define MOTOR_PWM_AF                GPIO_AF2_TIM4
#define MOTOR_PWM_TIMER_CLK()       __HAL_RCC_TIM4_CLK_ENABLE()

/**
 * @brief Piston A - Green Objects (Push Right)
 * Digital output
 */
#define PISTON_A_PORT               GPIOC
#define PISTON_A_PIN                GPIO_PIN_0
#define PISTON_A_GPIO_CLK()         __HAL_RCC_GPIOC_CLK_ENABLE()

/**
 * @brief Piston B - Blue Objects (Push Left)
 * Digital output
 */
#define PISTON_B_PORT               GPIOC
#define PISTON_B_PIN                GPIO_PIN_1
#define PISTON_B_GPIO_CLK()         __HAL_RCC_GPIOC_CLK_ENABLE()

/**
 * @brief Status LED (Green) - System Running
 * PA5 (on-board LED for STM32F407 Discovery)
 */
#define LED_STATUS_PORT             GPIOA
#define LED_STATUS_PIN              GPIO_PIN_5
#define LED_STATUS_GPIO_CLK()       __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Error LED (Red) - Emergency/Error State
 */
#define LED_ERROR_PORT              GPIOA
#define LED_ERROR_PIN               GPIO_PIN_6
#define LED_ERROR_GPIO_CLK()        __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Camera Trigger (Optional)
 * Pulse signal to trigger camera capture
 */
#define CAMERA_TRIGGER_PORT         GPIOB
#define CAMERA_TRIGGER_PIN          GPIO_PIN_7
#define CAMERA_TRIGGER_GPIO_CLK()   __HAL_RCC_GPIOB_CLK_ENABLE()

/* UART Configuration --------------------------------------------------------*/

/**
 * @brief Debug UART (for printf)
 * USART2: TX=PA2, RX=PA3
 */
#define DEBUG_UART                  USART2
#define DEBUG_UART_BAUDRATE         115200
#define DEBUG_UART_TX_PORT          GPIOA
#define DEBUG_UART_TX_PIN           GPIO_PIN_2
#define DEBUG_UART_RX_PORT          GPIOA
#define DEBUG_UART_RX_PIN           GPIO_PIN_3
#define DEBUG_UART_AF               GPIO_AF7_USART2
#define DEBUG_UART_CLK()            __HAL_RCC_USART2_CLK_ENABLE()
#define DEBUG_UART_GPIO_CLK()       __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Communication UART (inter-node communication)
 * USART1: TX=PA9, RX=PA10
 */
#define COMM_UART                   USART1
#define COMM_UART_BAUDRATE          115200
#define COMM_UART_TX_PORT           GPIOA
#define COMM_UART_TX_PIN            GPIO_PIN_9
#define COMM_UART_RX_PORT           GPIOA
#define COMM_UART_RX_PIN            GPIO_PIN_10
#define COMM_UART_AF                GPIO_AF7_USART1
#define COMM_UART_CLK()             __HAL_RCC_USART1_CLK_ENABLE()
#define COMM_UART_GPIO_CLK()        __HAL_RCC_GPIOA_CLK_ENABLE()

/* System Timing Parameters --------------------------------------------------*/

/**
 * @brief Conveyor Belt Parameters
 */
#define BELT_SPEED_MM_PER_SEC       100     /**< Belt speed in mm/s */
#define BELT_LENGTH_MM              1000    /**< Total belt length */

/**
 * @brief Camera to Piston Distance
 */
#define CAMERA_TO_PISTON_A_MM       200     /**< Distance to Piston A */
#define CAMERA_TO_PISTON_B_MM       200     /**< Distance to Piston B */

/**
 * @brief Calculate travel time (ms)
 */
#define TRAVEL_TIME_MS(distance_mm) ((distance_mm * 1000) / BELT_SPEED_MM_PER_SEC)

/**
 * @brief Piston Activation Time
 */
#define PISTON_ACTIVATION_TIME_MS   100     /**< Time piston stays extended */

/**
 * @brief Camera Frame Rate
 */
#define CAMERA_FRAME_PERIOD_MS      200     /**< Capture every 200ms (5 FPS) */

/**
 * @brief Emergency Response Time
 */
#define EMERGENCY_POLL_PERIOD_MS    10      /**< Check emergency button every 10ms */
#define EMERGENCY_RESPONSE_MAX_MS   10      /**< Max time to stop system */

/* FreeRTOS Task Priorities --------------------------------------------------*/

/**
 * @brief Task Priority Levels (0 = lowest, higher number = higher priority)
 * Based on Rate Monotonic Scheduling
 */
#define PRIORITY_IDLE               0       /**< FreeRTOS idle task */
#define PRIORITY_MONITOR            1       /**< Statistics monitor */
#define PRIORITY_CONVEYOR           2       /**< Conveyor control */
#define PRIORITY_HEARTBEAT          2       /**< Heartbeat */
#define PRIORITY_COMM_TX            2       /**< Communication TX */
#define PRIORITY_COMM_RX            3       /**< Communication RX */
#define PRIORITY_CAMERA_CAPTURE     3       /**< Camera capture */
#define PRIORITY_CAMERA_PROCESS     4       /**< Camera processing */
#define PRIORITY_PISTON_A           4       /**< Piston A control */
#define PRIORITY_PISTON_B           4       /**< Piston B control */
#define PRIORITY_SAFETY             10      /**< Emergency safety (HIGHEST) */

/* Queue and Semaphore Configuration -----------------------------------------*/

/**
 * @brief FreeRTOS Queue Sizes
 */
#define CAMERA_EVENT_QUEUE_SIZE     10      /**< Color detection events */
#define COMM_RX_QUEUE_SIZE          10      /**< Received messages */
#define COMM_TX_QUEUE_SIZE          10      /**< Messages to send */

/**
 * @brief Event Message Size
 */
#define EVENT_MESSAGE_SIZE          32      /**< Max bytes per message */

/* Debug and Monitoring ------------------------------------------------------*/

/**
 * @brief Debug Configuration
 */
#define ENABLE_DEBUG_UART           1       /**< Enable debug printf */
#define ENABLE_PERFORMANCE_MONITOR  1       /**< Enable CPU usage monitoring */
#define ENABLE_STACK_CHECKING       1       /**< Enable stack overflow checking */

/**
 * @brief Statistics Reporting Period
 */
#define STATS_REPORT_PERIOD_MS      10000   /**< Report every 10 seconds */

/* Hardware Abstraction Macros -----------------------------------------------*/

/**
 * @brief Read Emergency Button State
 * @retval true if pressed (active LOW), false otherwise
 */
#define IS_EMERGENCY_PRESSED()      (HAL_GPIO_ReadPin(EMERGENCY_BTN_PORT, EMERGENCY_BTN_PIN) == GPIO_PIN_RESET)

/**
 * @brief Control Status LED
 */
#define STATUS_LED_ON()             HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET)
#define STATUS_LED_OFF()            HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_RESET)
#define STATUS_LED_TOGGLE()         HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN)

/**
 * @brief Control Error LED
 */
#define ERROR_LED_ON()              HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET)
#define ERROR_LED_OFF()             HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET)
#define ERROR_LED_TOGGLE()          HAL_GPIO_TogglePin(LED_ERROR_PORT, LED_ERROR_PIN)

/**
 * @brief Control Pistons
 */
#define PISTON_A_ACTIVATE()         HAL_GPIO_WritePin(PISTON_A_PORT, PISTON_A_PIN, GPIO_PIN_SET)
#define PISTON_A_DEACTIVATE()       HAL_GPIO_WritePin(PISTON_A_PORT, PISTON_A_PIN, GPIO_PIN_RESET)
#define PISTON_B_ACTIVATE()         HAL_GPIO_WritePin(PISTON_B_PORT, PISTON_B_PIN, GPIO_PIN_SET)
#define PISTON_B_DEACTIVATE()       HAL_GPIO_WritePin(PISTON_B_PORT, PISTON_B_PIN, GPIO_PIN_RESET)

/* Memory Configuration ------------------------------------------------------*/

/**
 * @brief Linker Memory Regions (defined in .ld file)
 */
#define FLASH_START_ADDRESS         0x08000000  /**< Flash start */
#define FLASH_SIZE_KB               1024        /**< 1MB Flash (STM32F407VG) */
#define RAM_START_ADDRESS           0x20000000  /**< RAM start */
#define RAM_SIZE_KB                 128         /**< 128KB RAM (STM32F407VG) */

/**
 * @brief Heap and Stack Configuration
 */
#define MIN_STACK_SIZE              0x400   /**< Minimum stack size (1KB) */
#define MIN_HEAP_SIZE               0x200   /**< Minimum heap size (512B) */

/* DMA Configuration (Optional) ----------------------------------------------*/

/**
 * @brief DMA for UART (optional performance optimization)
 */
#define ENABLE_UART_DMA             0       /**< 0=Disabled, 1=Enabled */
#define DEBUG_UART_DMA_CHANNEL      DMA1_Stream6
#define COMM_UART_DMA_CHANNEL       DMA2_Stream7

/* Interrupt Priorities ------------------------------------------------------*/

/**
 * @brief NVIC Interrupt Priorities (0-15, lower = higher priority)
 * Must be >= configMAX_SYSCALL_INTERRUPT_PRIORITY for FreeRTOS compatibility
 */
#define NVIC_PRIORITY_EMERGENCY     5       /**< Emergency button */
#define NVIC_PRIORITY_TIMER         6       /**< System timers */
#define NVIC_PRIORITY_UART          7       /**< UART interrupts */
#define NVIC_PRIORITY_DMA           8       /**< DMA transfers */

/* Compile-Time Assertions ---------------------------------------------------*/

/**
 * @brief Validate configuration
 */
#if (configTOTAL_HEAP_SIZE > (RAM_SIZE_KB * 1024))
    #error "FreeRTOS heap size exceeds available RAM!"
#endif

#if (PRIORITY_SAFETY <= PRIORITY_CAMERA_PROCESS)
    #error "Safety task must have highest priority!"
#endif

/* Exported Functions --------------------------------------------------------*/

/**
 * @brief  Initialize system clock to maximum frequency
 * @retval None
 */
void SystemClock_Config(void);

/**
 * @brief  Initialize all peripherals
 * @retval None
 */
void SystemPeripherals_Init(void);

/**
 * @brief  Error handler
 * @retval None
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32_CONFIG_H */
