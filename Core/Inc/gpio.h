/**
 ******************************************************************************
 * @file           : gpio.h
 * @brief          : GPIO driver header for STM32 with FreeRTOS
 * @details        : Provides GPIO control for sensors, actuators, and LEDs
 ******************************************************************************
 * @attention
 *
 * Features:
 * - Digital I/O control (read/write pins)
 * - PWM output for motors and servos
 * - External interrupt support
 * - Thread-safe operations with FreeRTOS
 * - Hardware abstraction for conveyor belt system
 *
 * Compatible with:
 * - STM32F4xx, STM32F7xx, STM32H7xx series
 * - STM32CubeIDE HAL drivers
 * - FreeRTOS
 *
 ******************************************************************************
 */

#ifndef __GPIO_H
#define __GPIO_H

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
 * @brief GPIO Pin Configuration Structure
 */
typedef struct {
    GPIO_TypeDef *port;         /**< GPIO port (GPIOA, GPIOB, etc.) */
    uint16_t pin;               /**< GPIO pin number (GPIO_PIN_0, etc.) */
    uint32_t mode;              /**< GPIO mode (INPUT, OUTPUT, etc.) */
    uint32_t pull;              /**< Pull-up/down configuration */
    uint32_t speed;             /**< GPIO speed */
} GPIO_PinConfig_t;

/**
 * @brief GPIO Pin State
 */
typedef enum {
    GPIO_STATE_LOW = 0,         /**< Low level (0V) */
    GPIO_STATE_HIGH = 1         /**< High level (3.3V or 5V) */
} GPIO_PinState_t;

/**
 * @brief Hardware Pin Definitions for Conveyor Belt System
 */
typedef struct {
    /* Emergency Button (Input) */
    GPIO_TypeDef *emergency_port;
    uint16_t emergency_pin;

    /* Conveyor Motor Control (Output - PWM) */
    GPIO_TypeDef *motor_port;
    uint16_t motor_pin;
    TIM_HandleTypeDef *motor_timer;     /**< Timer for PWM */
    uint32_t motor_channel;             /**< PWM channel */

    /* Piston A Control (Output) */
    GPIO_TypeDef *piston_a_port;
    uint16_t piston_a_pin;

    /* Piston B Control (Output) */
    GPIO_TypeDef *piston_b_port;
    uint16_t piston_b_pin;

    /* Status LEDs (Output) */
    GPIO_TypeDef *led_status_port;
    uint16_t led_status_pin;            /**< System running LED */

    GPIO_TypeDef *led_error_port;
    uint16_t led_error_pin;             /**< Error/Emergency LED */

    /* Camera Trigger (Output - optional) */
    GPIO_TypeDef *camera_trigger_port;
    uint16_t camera_trigger_pin;
} HardwareConfig_t;

/* Exported constants --------------------------------------------------------*/

/* Default GPIO Ports and Pins - Customize based on your hardware */

/* Emergency Button - PA0 (Active LOW with pull-up) */
#define EMERGENCY_BUTTON_PORT       GPIOA
#define EMERGENCY_BUTTON_PIN        GPIO_PIN_0

/* Conveyor Motor PWM - PB6 (TIM4_CH1) */
#define MOTOR_PWM_PORT              GPIOB
#define MOTOR_PWM_PIN               GPIO_PIN_6
#define MOTOR_PWM_TIMER             TIM4
#define MOTOR_PWM_CHANNEL           TIM_CHANNEL_1

/* Piston A (GREEN) - PC0 */
#define PISTON_A_PORT               GPIOC
#define PISTON_A_PIN                GPIO_PIN_0

/* Piston B (BLUE) - PC1 */
#define PISTON_B_PORT               GPIOC
#define PISTON_B_PIN                GPIO_PIN_1

/* Status LED (GREEN) - PA5 */
#define LED_STATUS_PORT             GPIOA
#define LED_STATUS_PIN              GPIO_PIN_5

/* Error LED (RED) - PA6 */
#define LED_ERROR_PORT              GPIOA
#define LED_ERROR_PIN               GPIO_PIN_6

/* Camera Trigger - PB7 (optional) */
#define CAMERA_TRIGGER_PORT         GPIOB
#define CAMERA_TRIGGER_PIN          GPIO_PIN_7

/* PWM Configuration */
#define MOTOR_PWM_FREQUENCY         1000    /**< PWM frequency in Hz */
#define MOTOR_PWM_DUTY_MIN          0       /**< Minimum duty cycle (0%) */
#define MOTOR_PWM_DUTY_MAX          100     /**< Maximum duty cycle (100%) */

/* Timing Constants */
#define PISTON_ACTIVATION_TIME_MS   100     /**< Time to keep piston activated */
#define DEBOUNCE_DELAY_MS           50      /**< Button debounce delay */

/* Exported macro ------------------------------------------------------------*/

/**
 * @brief Read digital pin state
 */
#define GPIO_READ_PIN(port, pin)    HAL_GPIO_ReadPin(port, pin)

/**
 * @brief Write digital pin state
 */
#define GPIO_WRITE_PIN(port, pin, state) HAL_GPIO_WritePin(port, pin, state)

/**
 * @brief Toggle pin state
 */
#define GPIO_TOGGLE_PIN(port, pin)  HAL_GPIO_TogglePin(port, pin)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize all GPIO pins for the system
 * @param  config: Pointer to hardware configuration structure
 * @retval None
 */
void GPIO_InitAll(HardwareConfig_t *config);

/**
 * @brief  Initialize a single GPIO pin
 * @param  config: Pointer to pin configuration structure
 * @retval None
 */
void GPIO_InitPin(GPIO_PinConfig_t *config);

/**
 * @brief  De-initialize a GPIO pin
 * @param  port: GPIO port
 * @param  pin: GPIO pin number
 * @retval None
 */
void GPIO_DeInitPin(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  Read digital input pin
 * @param  port: GPIO port
 * @param  pin: GPIO pin number
 * @retval Pin state (GPIO_STATE_LOW or GPIO_STATE_HIGH)
 */
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief  Write digital output pin
 * @param  port: GPIO port
 * @param  pin: GPIO pin number
 * @param  state: Pin state to set
 * @retval None
 */
void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState_t state);

/**
 * @brief  Toggle digital output pin
 * @param  port: GPIO port
 * @param  pin: GPIO pin number
 * @retval None
 */
void GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

/* Hardware-specific functions -----------------------------------------------*/

/**
 * @brief  Read emergency button state (with debounce)
 * @retval true if button is pressed, false otherwise
 */
bool GPIO_ReadEmergencyButton(void);

/**
 * @brief  Set conveyor motor speed (0-100%)
 * @param  speed_percent: Speed percentage (0-100)
 * @retval None
 */
void GPIO_SetMotorSpeed(uint8_t speed_percent);

/**
 * @brief  Start conveyor motor
 * @retval None
 */
void GPIO_StartMotor(void);

/**
 * @brief  Stop conveyor motor immediately
 * @retval None
 */
void GPIO_StopMotor(void);

/**
 * @brief  Activate Piston A (for GREEN objects)
 * @param  duration_ms: Activation duration in milliseconds
 * @retval None
 */
void GPIO_ActivatePistonA(uint32_t duration_ms);

/**
 * @brief  Activate Piston B (for BLUE objects)
 * @param  duration_ms: Activation duration in milliseconds
 * @retval None
 */
void GPIO_ActivatePistonB(uint32_t duration_ms);

/**
 * @brief  Deactivate Piston A
 * @retval None
 */
void GPIO_DeactivatePistonA(void);

/**
 * @brief  Deactivate Piston B
 * @retval None
 */
void GPIO_DeactivatePistonB(void);

/**
 * @brief  Set status LED state
 * @param  state: LED state (on/off)
 * @retval None
 */
void GPIO_SetStatusLED(bool state);

/**
 * @brief  Set error LED state
 * @param  state: LED state (on/off)
 * @retval None
 */
void GPIO_SetErrorLED(bool state);

/**
 * @brief  Blink LED with specified pattern
 * @param  port: GPIO port
 * @param  pin: GPIO pin number
 * @param  count: Number of blinks
 * @param  delay_ms: Delay between blinks in milliseconds
 * @retval None
 */
void GPIO_BlinkLED(GPIO_TypeDef *port, uint16_t pin, uint8_t count, uint32_t delay_ms);

/**
 * @brief  Trigger camera capture (pulse signal)
 * @retval None
 */
void GPIO_TriggerCamera(void);

/* PWM Control functions -----------------------------------------------------*/

/**
 * @brief  Initialize PWM for motor control
 * @param  htim: Pointer to timer handle
 * @param  channel: PWM channel
 * @param  frequency_hz: PWM frequency in Hz
 * @retval HAL status
 */
HAL_StatusTypeDef GPIO_InitPWM(TIM_HandleTypeDef *htim, uint32_t channel,
                               uint32_t frequency_hz);

/**
 * @brief  Set PWM duty cycle
 * @param  htim: Pointer to timer handle
 * @param  channel: PWM channel
 * @param  duty_percent: Duty cycle percentage (0-100)
 * @retval None
 */
void GPIO_SetPWMDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_percent);

/**
 * @brief  Start PWM generation
 * @param  htim: Pointer to timer handle
 * @param  channel: PWM channel
 * @retval HAL status
 */
HAL_StatusTypeDef GPIO_StartPWM(TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief  Stop PWM generation
 * @param  htim: Pointer to timer handle
 * @param  channel: PWM channel
 * @retval HAL status
 */
HAL_StatusTypeDef GPIO_StopPWM(TIM_HandleTypeDef *htim, uint32_t channel);

/* Interrupt callback functions ----------------------------------------------*/

/**
 * @brief  GPIO EXTI callback (override this in your code)
 * @param  GPIO_Pin: Pin that triggered the interrupt
 * @retval None
 */
void GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;     /**< Timer for motor PWM */
extern HardwareConfig_t hw_config;  /**< Global hardware configuration */

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
