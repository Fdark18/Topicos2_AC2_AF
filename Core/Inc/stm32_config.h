/**
 ******************************************************************************
 * @file           : stm32_config.h
 * @brief          : STM32L053R8 configuration for FreeRTOS conveyor belt system
 * @details        : Hardware configuration, pinout definitions, and system parameters
 ******************************************************************************
 * @attention
 *
 * This file centralizes all STM32-specific configurations for the conveyor
 * belt system. Configured for STM32L053R8 (Nucleo-L053R8 board).
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
#include "stm32l0xx_hal.h"  // STM32L0 HAL library

/* MCU Configuration ---------------------------------------------------------*/

/**
 * @brief MCU Family Selection
 * STM32L053R8 - Nucleo-L053R8 board
 */
#define STM32L053xx

/**
 * @brief System Clock Configuration
 * STM32L053R8: Max 32 MHz (MSI, HSI16, or PLL)
 */
#define SYSTEM_CLOCK_MHZ            32      /**< System clock in MHz */
#define APB1_TIMER_CLOCK_MHZ        32      /**< APB1 timer clock */
#define APB2_TIMER_CLOCK_MHZ        32      /**< APB2 timer clock */

/* FreeRTOS Configuration ----------------------------------------------------*/

/**
 * @brief FreeRTOS Heap Size
 * STM32L053R8 has 8KB RAM - allocate conservatively
 */
#define configTOTAL_HEAP_SIZE       ((size_t)(4 * 1024))  /**< 4KB heap (50% of RAM) */

/**
 * @brief FreeRTOS Tick Rate
 */
#define configTICK_RATE_HZ          1000    /**< 1ms tick period */

/**
 * @brief Stack sizes for each task (in words, not bytes)
 * STM32 uses 32-bit words, so 128 words = 512 bytes
 * Reduced for STM32L053R8 limited RAM
 */
#define SAFETY_TASK_STACK_SIZE      128     /**< Safety node stack (512B) */
#define CAMERA_TASK_STACK_SIZE      256     /**< Camera node stack (1KB) */
#define PISTON_TASK_STACK_SIZE      128     /**< Piston node stack (512B) */
#define CONVEYOR_TASK_STACK_SIZE    128     /**< Conveyor node stack (512B) */
#define MONITOR_TASK_STACK_SIZE     256     /**< Monitor task stack (1KB) */

/* GPIO Pin Definitions ------------------------------------------------------*/

/**
 * @brief Emergency Stop Button
 * Active LOW with internal pull-up
 * Using USER Button (PC13) on Nucleo board
 */
#define EMERGENCY_BTN_PORT          GPIOC
#define EMERGENCY_BTN_PIN           GPIO_PIN_13
#define EMERGENCY_BTN_GPIO_CLK()    __HAL_RCC_GPIOC_CLK_ENABLE()

/**
 * @brief Conveyor Belt Motor (PWM Control)
 * TIM2 Channel 1 (PA0)
 */
#define MOTOR_PWM_PORT              GPIOA
#define MOTOR_PWM_PIN               GPIO_PIN_0
#define MOTOR_PWM_GPIO_CLK()        __HAL_RCC_GPIOA_CLK_ENABLE()
#define MOTOR_PWM_TIMER             TIM2
#define MOTOR_PWM_CHANNEL           TIM_CHANNEL_1
#define MOTOR_PWM_AF                GPIO_AF2_TIM2
#define MOTOR_PWM_TIMER_CLK()       __HAL_RCC_TIM2_CLK_ENABLE()

/**
 * @brief Piston A - Green Objects (Push Right)
 * Digital output (PA8)
 */
#define PISTON_A_PORT               GPIOA
#define PISTON_A_PIN                GPIO_PIN_8
#define PISTON_A_GPIO_CLK()         __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Piston B - Blue Objects (Push Left)
 * Digital output (PA9)
 */
#define PISTON_B_PORT               GPIOA
#define PISTON_B_PIN                GPIO_PIN_9
#define PISTON_B_GPIO_CLK()         __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Status LED (Green) - System Running
 * PA5 (on-board LED LD2 for Nucleo)
 */
#define LED_STATUS_PORT             GPIOA
#define LED_STATUS_PIN              GPIO_PIN_5
#define LED_STATUS_GPIO_CLK()       __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Error LED (Red) - Emergency/Error State
 * PB0 (external LED)
 */
#define LED_ERROR_PORT              GPIOB
#define LED_ERROR_PIN               GPIO_PIN_0
#define LED_ERROR_GPIO_CLK()        __HAL_RCC_GPIOB_CLK_ENABLE()

/**
 * @brief Camera Trigger (Optional)
 * Pulse signal to trigger camera capture (PA10)
 */
#define CAMERA_TRIGGER_PORT         GPIOA
#define CAMERA_TRIGGER_PIN          GPIO_PIN_10
#define CAMERA_TRIGGER_GPIO_CLK()   __HAL_RCC_GPIOA_CLK_ENABLE()

/* UART Configuration --------------------------------------------------------*/

/**
 * @brief Debug UART (for printf)
 * USART2: TX=PA2, RX=PA3 (connected to ST-Link on Nucleo)
 */
#define DEBUG_UART                  USART2
#define DEBUG_UART_BAUDRATE         115200
#define DEBUG_UART_TX_PORT          GPIOA
#define DEBUG_UART_TX_PIN           GPIO_PIN_2
#define DEBUG_UART_RX_PORT          GPIOA
#define DEBUG_UART_RX_PIN           GPIO_PIN_3
#define DEBUG_UART_AF               GPIO_AF4_USART2
#define DEBUG_UART_CLK()            __HAL_RCC_USART2_CLK_ENABLE()
#define DEBUG_UART_GPIO_CLK()       __HAL_RCC_GPIOA_CLK_ENABLE()

/**
 * @brief Communication UART (inter-node communication)
 * USART1: TX=PA9, RX=PA10 (Note: conflicts with PISTON_B and CAMERA_TRIGGER)
 * Alternative: LPUART1 on different pins
 */
#define COMM_UART                   USART1
#define COMM_UART_BAUDRATE          115200
#define COMM_UART_TX_PORT           GPIOB
#define COMM_UART_TX_PIN            GPIO_PIN_6
#define COMM_UART_RX_PORT           GPIOB
#define COMM_UART_RX_PIN            GPIO_PIN_7
#define COMM_UART_AF                GPIO_AF0_USART1
#define COMM_UART_CLK()             __HAL_RCC_USART1_CLK_ENABLE()
#define COMM_UART_GPIO_CLK()        __HAL_RCC_GPIOB_CLK_ENABLE()

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
#define PRIORITY_SAFETY             5       /**< Emergency safety (HIGHEST) */

/* Queue and Semaphore Configuration -----------------------------------------*/

/**
 * @brief FreeRTOS Queue Sizes
 */
#define CAMERA_EVENT_QUEUE_SIZE     5       /**< Color detection events (reduced for RAM) */
#define COMM_RX_QUEUE_SIZE          5       /**< Received messages */
#define COMM_TX_QUEUE_SIZE          5       /**< Messages to send */

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
 * STM32L053R8 specifications
 */
#define FLASH_START_ADDRESS         0x08000000  /**< Flash start */
#define FLASH_SIZE_KB               64          /**< 64KB Flash */
#define RAM_START_ADDRESS           0x20000000  /**< RAM start */
#define RAM_SIZE_KB                 8           /**< 8KB RAM */

/**
 * @brief Heap and Stack Configuration
 */
#define MIN_STACK_SIZE              0x200   /**< Minimum stack size (512B) */
#define MIN_HEAP_SIZE               0x100   /**< Minimum heap size (256B) */

/* Interrupt Priorities ------------------------------------------------------*/

/**
 * @brief NVIC Interrupt Priorities (0-3 for STM32L0, lower = higher priority)
 * Must be >= configMAX_SYSCALL_INTERRUPT_PRIORITY for FreeRTOS compatibility
 */
#define NVIC_PRIORITY_EMERGENCY     1       /**< Emergency button */
#define NVIC_PRIORITY_TIMER         2       /**< System timers */
#define NVIC_PRIORITY_UART          3       /**< UART interrupts */

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
 * @brief  Initialize system clock to maximum frequency (32MHz)
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
