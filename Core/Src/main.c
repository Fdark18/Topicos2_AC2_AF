/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - FreeRTOS Conveyor System
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * Sistema de Esteira Industrial com FreeRTOS para STM32L053R8 Nucleo
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32_config.h"
#include "base_node.h"
#include "camera_node.h"
#include "piston_node.h"
#include "conveyor_node.h"
#include "safety_node.h"
#include "timing_config.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;

/* Node instances */
static SafetyNode_t safety;
static ConveyorNode_t conveyor;
static CameraNode_t camera;
static PistonNode_t piston_a;
static PistonNode_t piston_b;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Init(void);
void Error_Handler(void);

/* FreeRTOS task prototypes */
static void SimulationTask(void *pvParameters);
static void StatsTask(void *pvParameters);
static void BlinkTask(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM2_Init();

  /* Print startup banner */
  printf("\r\n");
  printf("╔════════════════════════════════════════════════════════════╗\r\n");
  printf("║   SISTEMA DE ESTEIRA INDUSTRIAL - STM32L053R8             ║\r\n");
  printf("║   Sistema Distribuído de Tempo Real com FreeRTOS          ║\r\n");
  printf("╚════════════════════════════════════════════════════════════╝\r\n");
  printf("\r\n");

  /* Initialize system nodes */
  printf("[INIT] Inicializando nós do sistema...\r\n");

  /* 1. Initialize Safety Node (HIGHEST PRIORITY) */
  if (!SafetyNode_Init(&safety, "safety_master", 0)) {
    printf("[ERROR] Falha ao inicializar Safety Node\r\n");
    Error_Handler();
  }

  /* 2. Initialize Conveyor Node */
  if (!ConveyorNode_Init(&conveyor, "conveyor_belt", 0, BELT_SPEED_MM_S)) {
    printf("[ERROR] Falha ao inicializar Conveyor Node\r\n");
    Error_Handler();
  }

  /* 3. Initialize Piston A (GREEN -> RIGHT) */
  if (!PistonNode_CreatePistonA(&piston_a)) {
    printf("[ERROR] Falha ao inicializar Piston A\r\n");
    Error_Handler();
  }

  /* 4. Initialize Piston B (BLUE -> LEFT) */
  if (!PistonNode_CreatePistonB(&piston_b)) {
    printf("[ERROR] Falha ao inicializar Piston B\r\n");
    Error_Handler();
  }

  /* 5. Initialize Camera Node */
  uint16_t piston_ports[] = {5002, 5003};
  if (!CameraNode_Init(&camera, "camera_1", piston_ports, 2)) {
    printf("[ERROR] Falha ao inicializar Camera Node\r\n");
    Error_Handler();
  }

  printf("[INIT] Todos os nós inicializados com sucesso\r\n\r\n");

  /* Start all nodes */
  printf("[MAIN] Iniciando nós do sistema...\r\n");

  if (!SafetyNode_Start(&safety)) {
    printf("[ERROR] Falha ao iniciar Safety Node\r\n");
    Error_Handler();
  }

  if (!ConveyorNode_Start(&conveyor)) {
    printf("[ERROR] Falha ao iniciar Conveyor Node\r\n");
    Error_Handler();
  }

  if (!PistonNode_Start(&piston_a)) {
    printf("[ERROR] Falha ao iniciar Piston A\r\n");
    Error_Handler();
  }

  if (!PistonNode_Start(&piston_b)) {
    printf("[ERROR] Falha ao iniciar Piston B\r\n");
    Error_Handler();
  }

  if (!CameraNode_Start(&camera)) {
    printf("[ERROR] Falha ao iniciar Camera Node\r\n");
    Error_Handler();
  }

  /* Start conveyor belt */
  ConveyorNode_Run(&conveyor);

  printf("[MAIN] Sistema iniciado com sucesso\r\n\r\n");

  /* Create auxiliary tasks */
  printf("[MAIN] Criando tarefas auxiliares...\r\n");

  /* LED blink task (low priority) */
  xTaskCreate(BlinkTask, "Blink", 128, NULL, 1, NULL);

  /* Simulation task (if needed) */
  xTaskCreate(SimulationTask, "Simulation", 256, NULL, 2, NULL);

  /* Statistics task */
  xTaskCreate(StatsTask, "Statistics", 512, NULL, 1, NULL);

  printf("[MAIN] Tarefas criadas. Iniciando scheduler FreeRTOS...\r\n\r\n");

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function (for PWM)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;  /* 32 MHz / 32 = 1 MHz */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;   /* 1 MHz / 1000 = 1 kHz PWM */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;  /* 50% duty cycle */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_PIN|PISTON_A_PIN|PISTON_B_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET);

  /*Configure Emergency Button (PC13 - USER button) */
  GPIO_InitStruct.Pin = EMERGENCY_BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EMERGENCY_BTN_PORT, &GPIO_InitStruct);

  /*Configure Status LED (PA5 - LD2) */
  GPIO_InitStruct.Pin = LED_STATUS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_PORT, &GPIO_InitStruct);

  /*Configure Error LED (PB0) */
  GPIO_InitStruct.Pin = LED_ERROR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ERROR_PORT, &GPIO_InitStruct);

  /*Configure Piston A (PA8) */
  GPIO_InitStruct.Pin = PISTON_A_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PISTON_A_PORT, &GPIO_InitStruct);

  /*Configure Piston B (PA9) */
  GPIO_InitStruct.Pin = PISTON_B_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PISTON_B_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Blink LED Task
 * @param  pvParameters: Not used
 * @retval None
 */
static void BlinkTask(void *pvParameters)
{
  while (1)
  {
    STATUS_LED_TOGGLE();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
 * @brief  Simulation Task (simulates color detections)
 * @param  pvParameters: Not used
 * @retval None
 */
static void SimulationTask(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(5000); // 5 seconds
  uint8_t sequence = 0;

  printf("[SIMULACAO] Tarefa iniciada - Simulando detecções a cada 5s\r\n");
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2s before starting

  while (1)
  {
    if (sequence % 2 == 0) {
      printf("[SIMULACAO] ===== Simulando detecção de VERDE =====\r\n");
      CameraNode_SimulateDetection(&camera, COLOR_GREEN, 500, 0.85f);
    } else {
      printf("[SIMULACAO] ===== Simulando detecção de AZUL =====\r\n");
      CameraNode_SimulateDetection(&camera, COLOR_BLUE, 450, 0.90f);
    }

    sequence++;
    vTaskDelay(xFrequency);
  }
}

/**
 * @brief  Statistics Task
 * @param  pvParameters: Not used
 * @retval None
 */
static void StatsTask(void *pvParameters)
{
  const TickType_t xFrequency = pdMS_TO_TICKS(15000); // 15 seconds

  vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10s before first print

  while (1)
  {
    printf("\r\n");
    printf("╔════════════════════════════════════════════════════════════╗\r\n");
    printf("║         RELATORIO DE ESTATISTICAS DO SISTEMA              ║\r\n");
    printf("╚════════════════════════════════════════════════════════════╝\r\n");

    SafetyNode_PrintStats(&safety);
    ConveyorNode_PrintStats(&conveyor);
    CameraNode_PrintStats(&camera);
    PistonNode_PrintStats(&piston_a);
    PistonNode_PrintStats(&piston_b);

    vTaskDelay(xFrequency);
  }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

/**
 * @brief  FreeRTOS malloc failed hook
 * @retval None
 */
void vApplicationMallocFailedHook(void)
{
  printf("[FREERTOS] ERRO: Falha ao alocar memória!\r\n");
  Error_Handler();
}

/**
 * @brief  FreeRTOS stack overflow hook
 * @param  xTask: Task handle
 * @param  pcTaskName: Task name
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  printf("[FREERTOS] ERRO: Stack overflow na tarefa: %s\r\n", pcTaskName);
  Error_Handler();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  ERROR_LED_ON();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
