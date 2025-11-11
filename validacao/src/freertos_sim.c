#include "freertos_sim.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#include <unistd.h>
#endif

typedef struct freertos_task {
    pthread_t thread;
    TaskFunction_t function;
    void *params;
} freertos_task_t;

typedef struct freertos_queue {
    size_t item_size;
    size_t length;
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t count;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} freertos_queue_t;

typedef struct freertos_semaphore {
    pthread_mutex_t mutex;
} freertos_semaphore_t;

static uint64_t g_start_tick_ms = 0;
static pthread_key_t g_task_key;
static pthread_once_t g_task_key_once = PTHREAD_ONCE_INIT;

static void freertos_task_key_init(void) {
    pthread_key_create(&g_task_key, free);
}

static uint64_t freertos_sim_get_time_ms(void) {
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000ULL) + (ts.tv_nsec / 1000000ULL);
#endif
}

void freertos_sim_init(void) {
    if (g_start_tick_ms == 0) {
        g_start_tick_ms = freertos_sim_get_time_ms();
    }
}

void freertos_sim_shutdown(void) {
    g_start_tick_ms = 0;
}

static void *freertos_task_runner(void *arg) {
    pthread_once(&g_task_key_once, freertos_task_key_init);
    pthread_setspecific(g_task_key, arg);
    freertos_task_t *task = (freertos_task_t *)arg;
    task->function(task->params);
    return NULL;
}

BaseType_t xTaskCreate(TaskFunction_t pxTaskCode,
                       const char *pcName,
                       uint16_t usStackDepth,
                       void *pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t *pxCreatedTask) {
    (void)pcName;
    (void)usStackDepth;
    (void)uxPriority;

    freertos_task_t *task = (freertos_task_t *)malloc(sizeof(freertos_task_t));
    if (task == NULL) {
        return pdFAIL;
    }

    task->function = pxTaskCode;
    task->params = pvParameters;

    if (pthread_create(&task->thread, NULL, freertos_task_runner, task) != 0) {
        free(task);
        return pdFAIL;
    }

    pthread_detach(task->thread);

    if (pxCreatedTask != NULL) {
        *pxCreatedTask = task;
    }

    return pdPASS;
}

void vTaskDelete(TaskHandle_t xTask) {
    if (xTask == NULL) {
        pthread_exit(NULL);
    } else {
        // Not implemented: deleting outras tarefas não é necessário para a validação.
    }
}

void vTaskDelay(TickType_t xTicksToDelay) {
#ifdef _WIN32
    Sleep(xTicksToDelay);
#else
    usleep((useconds_t)xTicksToDelay * 1000U);
#endif
}

TickType_t xTaskGetTickCount(void) {
    if (g_start_tick_ms == 0) {
        freertos_sim_init();
    }
    uint64_t now = freertos_sim_get_time_ms();
    return (TickType_t)(now - g_start_tick_ms);
}

void vTaskStartScheduler(void) {
    // Nothing to do; pthreads are created immediately.
}

static void queue_init_sync_objects(freertos_queue_t *queue) {
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
}

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize) {
    if (uxQueueLength == 0 || uxItemSize == 0) {
        return NULL;
    }

    freertos_queue_t *queue = (freertos_queue_t *)calloc(1, sizeof(freertos_queue_t));
    if (queue == NULL) {
        return NULL;
    }

    queue->buffer = (uint8_t *)malloc(uxQueueLength * uxItemSize);
    if (queue->buffer == NULL) {
        free(queue);
        return NULL;
    }

    queue->item_size = uxItemSize;
    queue->length = uxQueueLength;
    queue_init_sync_objects(queue);
    return queue;
}

static void queue_wait_full(freertos_queue_t *queue) {
    pthread_cond_wait(&queue->not_full, &queue->mutex);
}

static void queue_wait_empty(freertos_queue_t *queue) {
    pthread_cond_wait(&queue->not_empty, &queue->mutex);
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL || pvItemToQueue == NULL) {
        return pdFAIL;
    }

    pthread_mutex_lock(&queue->mutex);
    while (queue->count == queue->length) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&queue->mutex);
            return pdFAIL;
        }
        queue_wait_full(queue);
    }

    memcpy(queue->buffer + (queue->tail * queue->item_size), pvItemToQueue, queue->item_size);
    queue->tail = (queue->tail + 1) % queue->length;
    queue->count++;
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL || pvBuffer == NULL) {
        return pdFAIL;
    }

    pthread_mutex_lock(&queue->mutex);
    while (queue->count == 0) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&queue->mutex);
            return pdFAIL;
        }
        queue_wait_empty(queue);
    }

    memcpy(pvBuffer, queue->buffer + (queue->head * queue->item_size), queue->item_size);
    queue->head = (queue->head + 1) % queue->length;
    queue->count--;
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
    return pdPASS;
}

void vQueueDelete(QueueHandle_t xQueue) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL) {
        return;
    }
    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);
    free(queue->buffer);
    free(queue);
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)malloc(sizeof(freertos_semaphore_t));
    if (sem == NULL) {
        return NULL;
    }
    pthread_mutex_init(&sem->mutex, NULL);
    return sem;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return pdFAIL;
    }
    pthread_mutex_lock(&sem->mutex);
    return pdPASS;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return pdFAIL;
    }
    pthread_mutex_unlock(&sem->mutex);
    return pdPASS;
}

void vSemaphoreDelete(SemaphoreHandle_t xSemaphore) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return;
    }
    pthread_mutex_destroy(&sem->mutex);
    free(sem);
}
