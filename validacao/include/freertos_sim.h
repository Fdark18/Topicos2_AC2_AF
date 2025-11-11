#ifndef VALIDACAO_FREERTOS_SIM_H
#define VALIDACAO_FREERTOS_SIM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;

typedef void (*TaskFunction_t)(void *);

typedef struct freertos_task *TaskHandle_t;
typedef struct freertos_queue *QueueHandle_t;
typedef struct freertos_semaphore *SemaphoreHandle_t;

#define pdFALSE            ((BaseType_t)0)
#define pdTRUE             ((BaseType_t)1)
#define pdPASS             (pdTRUE)
#define pdFAIL             (pdFALSE)
#define portTICK_PERIOD_MS ((TickType_t)1)
#define configTICK_RATE_HZ (1000U)
#define pdMS_TO_TICKS(ms)  ((TickType_t)(ms))
#define portMAX_DELAY      ((TickType_t)0xFFFFFFFFUL)

void freertos_sim_init(void);
void freertos_sim_shutdown(void);

BaseType_t xTaskCreate(TaskFunction_t pxTaskCode,
                       const char *pcName,
                       uint16_t usStackDepth,
                       void *pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t *pxCreatedTask);
void vTaskDelete(TaskHandle_t xTask);
void vTaskDelay(TickType_t xTicksToDelay);
TickType_t xTaskGetTickCount(void);
void vTaskStartScheduler(void);

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);
void vQueueDelete(QueueHandle_t xQueue);

SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);
void vSemaphoreDelete(SemaphoreHandle_t xSemaphore);

#ifdef __cplusplus
}
#endif

#endif /* VALIDACAO_FREERTOS_SIM_H */
