/**
 ******************************************************************************
 * @file           : gpio.c
 * @brief          : GPIO driver implementation for STM32 with FreeRTOS
 * @details        : Thread-safe GPIO control for conveyor belt system
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private defines -----------------------------------------------------------*/
#define PWM_TIMER_CLOCK         84000000    /**< APB1 timer clock (adjust for your MCU) */
#define PWM_PRESCALER           83          /**< Prescaler for 1MHz timer clock */
#define PWM_PERIOD_1KHZ         1000        /**< Period for 1kHz PWM */

/* Private variables ---------------------------------------------------------*/
static SemaphoreHandle_t gpio_mutex = NULL;
static bool emergency_state = false;
static uint32_t last_debounce_time = 0;

/* Global hardware configuration */
HardwareConfig_t hw_config;

/* Timer handle for motor PWM */
TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_ConfigurePin(GPIO_TypeDef *port, uint16_t pin, uint32_t mode,
                              uint32_t pull, uint32_t speed);
static bool GPIO_Debounce(GPIO_TypeDef *port, uint16_t pin, uint32_t delay_ms);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Initialize all GPIO pins for the system
 */
void GPIO_InitAll(HardwareConfig_t *config)
{
    /* Create mutex for thread-safe GPIO operations */
    if (gpio_mutex == NULL)
    {
        gpio_mutex = xSemaphoreCreateMutex();
    }

    /* Enable GPIO Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Store hardware configuration */
    if (config != NULL)
    {
        hw_config = *config;
    }
    else
    {
        /* Use default configuration */
        hw_config.emergency_port = EMERGENCY_BUTTON_PORT;
        hw_config.emergency_pin = EMERGENCY_BUTTON_PIN;
        hw_config.motor_port = MOTOR_PWM_PORT;
        hw_config.motor_pin = MOTOR_PWM_PIN;
        hw_config.motor_timer = &htim4;
        hw_config.motor_channel = MOTOR_PWM_CHANNEL;
        hw_config.piston_a_port = PISTON_A_PORT;
        hw_config.piston_a_pin = PISTON_A_PIN;
        hw_config.piston_b_port = PISTON_B_PORT;
        hw_config.piston_b_pin = PISTON_B_PIN;
        hw_config.led_status_port = LED_STATUS_PORT;
        hw_config.led_status_pin = LED_STATUS_PIN;
        hw_config.led_error_port = LED_ERROR_PORT;
        hw_config.led_error_pin = LED_ERROR_PIN;
        hw_config.camera_trigger_port = CAMERA_TRIGGER_PORT;
        hw_config.camera_trigger_pin = CAMERA_TRIGGER_PIN;
    }

    /* Configure Emergency Button (Input with Pull-up) */
    GPIO_ConfigurePin(hw_config.emergency_port, hw_config.emergency_pin,
                     GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);

    /* Configure Piston Outputs */
    GPIO_ConfigurePin(hw_config.piston_a_port, hw_config.piston_a_pin,
                     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
    GPIO_ConfigurePin(hw_config.piston_b_port, hw_config.piston_b_pin,
                     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);

    /* Configure LED Outputs */
    GPIO_ConfigurePin(hw_config.led_status_port, hw_config.led_status_pin,
                     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
    GPIO_ConfigurePin(hw_config.led_error_port, hw_config.led_error_pin,
                     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);

    /* Configure Camera Trigger */
    GPIO_ConfigurePin(hw_config.camera_trigger_port, hw_config.camera_trigger_pin,
                     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);

    /* Initialize all outputs to LOW */
    GPIO_WritePin(hw_config.piston_a_port, hw_config.piston_a_pin, GPIO_STATE_LOW);
    GPIO_WritePin(hw_config.piston_b_port, hw_config.piston_b_pin, GPIO_STATE_LOW);
    GPIO_WritePin(hw_config.led_status_port, hw_config.led_status_pin, GPIO_STATE_LOW);
    GPIO_WritePin(hw_config.led_error_port, hw_config.led_error_pin, GPIO_STATE_LOW);
    GPIO_WritePin(hw_config.camera_trigger_port, hw_config.camera_trigger_pin, GPIO_STATE_LOW);

    /* Initialize PWM for motor control */
    GPIO_InitPWM(&htim4, MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQUENCY);
}

/**
 * @brief  Initialize a single GPIO pin
 */
void GPIO_InitPin(GPIO_PinConfig_t *config)
{
    if (config == NULL || config->port == NULL)
    {
        return;
    }

    GPIO_ConfigurePin(config->port, config->pin, config->mode,
                     config->pull, config->speed);
}

/**
 * @brief  De-initialize a GPIO pin
 */
void GPIO_DeInitPin(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_DeInit(port, pin);
}

/**
 * @brief  Read digital input pin
 */
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (GPIO_PinState_t)HAL_GPIO_ReadPin(port, pin);
}

/**
 * @brief  Write digital output pin
 */
void GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState_t state)
{
    HAL_GPIO_WritePin(port, pin, (GPIO_PinState)state);
}

/**
 * @brief  Toggle digital output pin
 */
void GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    HAL_GPIO_TogglePin(port, pin);
}

/* Hardware-specific functions -----------------------------------------------*/

/**
 * @brief  Read emergency button state (with debounce)
 */
bool GPIO_ReadEmergencyButton(void)
{
    /* Emergency button is active LOW */
    bool current_state = (GPIO_ReadPin(hw_config.emergency_port,
                                       hw_config.emergency_pin) == GPIO_STATE_LOW);

    /* Apply debounce */
    if (GPIO_Debounce(hw_config.emergency_port, hw_config.emergency_pin,
                     DEBOUNCE_DELAY_MS))
    {
        emergency_state = current_state;
    }

    return emergency_state;
}

/**
 * @brief  Set conveyor motor speed (0-100%)
 */
void GPIO_SetMotorSpeed(uint8_t speed_percent)
{
    /* Limit speed to valid range */
    if (speed_percent > 100)
    {
        speed_percent = 100;
    }

    /* Set PWM duty cycle */
    GPIO_SetPWMDuty(&htim4, MOTOR_PWM_CHANNEL, speed_percent);
}

/**
 * @brief  Start conveyor motor
 */
void GPIO_StartMotor(void)
{
    GPIO_StartPWM(&htim4, MOTOR_PWM_CHANNEL);
    GPIO_SetMotorSpeed(100);  // Full speed by default
}

/**
 * @brief  Stop conveyor motor immediately
 */
void GPIO_StopMotor(void)
{
    GPIO_SetMotorSpeed(0);
    GPIO_StopPWM(&htim4, MOTOR_PWM_CHANNEL);
}

/**
 * @brief  Activate Piston A (for GREEN objects)
 */
void GPIO_ActivatePistonA(uint32_t duration_ms)
{
    /* Activate piston */
    GPIO_WritePin(hw_config.piston_a_port, hw_config.piston_a_pin, GPIO_STATE_HIGH);

    /* Wait for specified duration */
    if (duration_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        GPIO_DeactivatePistonA();
    }
}

/**
 * @brief  Activate Piston B (for BLUE objects)
 */
void GPIO_ActivatePistonB(uint32_t duration_ms)
{
    /* Activate piston */
    GPIO_WritePin(hw_config.piston_b_port, hw_config.piston_b_pin, GPIO_STATE_HIGH);

    /* Wait for specified duration */
    if (duration_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        GPIO_DeactivatePistonB();
    }
}

/**
 * @brief  Deactivate Piston A
 */
void GPIO_DeactivatePistonA(void)
{
    GPIO_WritePin(hw_config.piston_a_port, hw_config.piston_a_pin, GPIO_STATE_LOW);
}

/**
 * @brief  Deactivate Piston B
 */
void GPIO_DeactivatePistonB(void)
{
    GPIO_WritePin(hw_config.piston_b_port, hw_config.piston_b_pin, GPIO_STATE_LOW);
}

/**
 * @brief  Set status LED state
 */
void GPIO_SetStatusLED(bool state)
{
    GPIO_WritePin(hw_config.led_status_port, hw_config.led_status_pin,
                 state ? GPIO_STATE_HIGH : GPIO_STATE_LOW);
}

/**
 * @brief  Set error LED state
 */
void GPIO_SetErrorLED(bool state)
{
    GPIO_WritePin(hw_config.led_error_port, hw_config.led_error_pin,
                 state ? GPIO_STATE_HIGH : GPIO_STATE_LOW);
}

/**
 * @brief  Blink LED with specified pattern
 */
void GPIO_BlinkLED(GPIO_TypeDef *port, uint16_t pin, uint8_t count, uint32_t delay_ms)
{
    for (uint8_t i = 0; i < count; i++)
    {
        GPIO_TogglePin(port, pin);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        GPIO_TogglePin(port, pin);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

/**
 * @brief  Trigger camera capture (pulse signal)
 */
void GPIO_TriggerCamera(void)
{
    /* Generate a 10ms pulse */
    GPIO_WritePin(hw_config.camera_trigger_port, hw_config.camera_trigger_pin,
                 GPIO_STATE_HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    GPIO_WritePin(hw_config.camera_trigger_port, hw_config.camera_trigger_pin,
                 GPIO_STATE_LOW);
}

/* PWM Control functions -----------------------------------------------------*/

/**
 * @brief  Initialize PWM for motor control
 */
HAL_StatusTypeDef GPIO_InitPWM(TIM_HandleTypeDef *htim, uint32_t channel,
                               uint32_t frequency_hz)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* Enable timer clock */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Calculate timer parameters */
    uint32_t prescaler = PWM_PRESCALER;
    uint32_t period = (PWM_TIMER_CLOCK / ((prescaler + 1) * frequency_hz)) - 1;

    /* Configure timer base */
    htim->Instance = TIM4;
    htim->Init.Prescaler = prescaler;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Configure PWM channel */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Start with 0% duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  Set PWM duty cycle
 */
void GPIO_SetPWMDuty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_percent)
{
    /* Limit duty cycle to valid range */
    if (duty_percent > 100)
    {
        duty_percent = 100;
    }

    /* Calculate compare value */
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t pulse = (period * duty_percent) / 100;

    /* Set compare value */
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/**
 * @brief  Start PWM generation
 */
HAL_StatusTypeDef GPIO_StartPWM(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief  Stop PWM generation
 */
HAL_StatusTypeDef GPIO_StopPWM(TIM_HandleTypeDef *htim, uint32_t channel)
{
    return HAL_TIM_PWM_Stop(htim, channel);
}

/* Interrupt callback functions ----------------------------------------------*/

/**
 * @brief  GPIO EXTI callback (weak implementation)
 */
__weak void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIO_Pin);

    /* NOTE: This function should be overridden in user code */
}

/**
 * @brief  HAL GPIO EXTI callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIO_EXTI_Callback(GPIO_Pin);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configure a GPIO pin
 */
static void GPIO_ConfigurePin(GPIO_TypeDef *port, uint16_t pin, uint32_t mode,
                              uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;

    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/**
 * @brief  Debounce GPIO input
 */
static bool GPIO_Debounce(GPIO_TypeDef *port, uint16_t pin, uint32_t delay_ms)
{
    uint32_t current_time = HAL_GetTick();

    /* Check if debounce period has elapsed */
    if ((current_time - last_debounce_time) > delay_ms)
    {
        last_debounce_time = current_time;
        return true;
    }

    return false;
}

/* MSP Initialization functions (called by HAL) ------------------------------*/

/**
 * @brief  TIM4 MSP Initialization (for PWM)
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM4)
    {
        /* Enable timer clock */
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* Enable GPIO clock */
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure GPIO pin for PWM output (PB6 - TIM4_CH1) */
        GPIO_InitStruct.Pin = MOTOR_PWM_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;

        HAL_GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStruct);
    }
}

/**
 * @brief  TIM4 MSP De-Initialization
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        /* Disable timer clock */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /* De-initialize GPIO */
        HAL_GPIO_DeInit(MOTOR_PWM_PORT, MOTOR_PWM_PIN);
    }
}
