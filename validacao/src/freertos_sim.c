#include "freertos_sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#endif

typedef struct freertos_task {
#ifdef _WIN32
    HANDLE thread;
#else
    pthread_t thread;
#endif
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
#ifdef _WIN32
    CRITICAL_SECTION mutex;
    CONDITION_VARIABLE not_empty;
    CONDITION_VARIABLE not_full;
#else
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
#endif
} freertos_queue_t;

typedef struct freertos_semaphore {
#ifdef _WIN32
    CRITICAL_SECTION mutex;
#else
    pthread_mutex_t mutex;
#endif
} freertos_semaphore_t;

static uint64_t g_start_tick_ms = 0;

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

#ifdef _WIN32
static DWORD WINAPI freertos_task_runner(LPVOID arg) {
    freertos_task_t *task = (freertos_task_t *)arg;
    task->function(task->params);
    free(task);
    return 0;
}
#else
static void *freertos_task_runner(void *arg) {
    freertos_task_t *task = (freertos_task_t *)arg;
    task->function(task->params);
    free(task);
    return NULL;
}
#endif

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

#ifdef _WIN32
    task->thread = CreateThread(NULL, 0, freertos_task_runner, task, 0, NULL);
    if (task->thread == NULL) {
        free(task);
        return pdFAIL;
    }
    CloseHandle(task->thread); // Detach
#else
    if (pthread_create(&task->thread, NULL, freertos_task_runner, task) != 0) {
        free(task);
        return pdFAIL;
    }
    pthread_detach(task->thread);
#endif

    if (pxCreatedTask != NULL) {
        *pxCreatedTask = task;
    }

    return pdPASS;
}

void vTaskDelete(TaskHandle_t xTask) {
    if (xTask == NULL) {
#ifdef _WIN32
        ExitThread(0);
#else
        pthread_exit(NULL);
#endif
    } else {
        // No-op: tasks auto-clean on exit in the simulator.
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
    // Nothing to do; threads start immediately.
}

static void queue_init_sync_objects(freertos_queue_t *queue) {
#ifdef _WIN32
    InitializeCriticalSection(&queue->mutex);
    InitializeConditionVariable(&queue->not_empty);
    InitializeConditionVariable(&queue->not_full);
#else
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
#endif
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

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL || pvItemToQueue == NULL) {
        return pdFAIL;
    }

#ifdef _WIN32
    EnterCriticalSection(&queue->mutex);
    while (queue->count == queue->length) {
        if (xTicksToWait == 0) {
            LeaveCriticalSection(&queue->mutex);
            return pdFAIL;
        }
        SleepConditionVariableCS(&queue->not_full, &queue->mutex, INFINITE);
    }
#else
    pthread_mutex_lock(&queue->mutex);
    while (queue->count == queue->length) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&queue->mutex);
            return pdFAIL;
        }
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }
#endif

    memcpy(queue->buffer + (queue->tail * queue->item_size), pvItemToQueue, queue->item_size);
    queue->tail = (queue->tail + 1) % queue->length;
    queue->count++;

#ifdef _WIN32
    WakeConditionVariable(&queue->not_empty);
    LeaveCriticalSection(&queue->mutex);
#else
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
#endif

    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL || pvBuffer == NULL) {
        return pdFAIL;
    }

#ifdef _WIN32
    EnterCriticalSection(&queue->mutex);
    while (queue->count == 0) {
        if (xTicksToWait == 0) {
            LeaveCriticalSection(&queue->mutex);
            return pdFAIL;
        }
        SleepConditionVariableCS(&queue->not_empty, &queue->mutex, INFINITE);
    }
#else
    pthread_mutex_lock(&queue->mutex);
    while (queue->count == 0) {
        if (xTicksToWait == 0) {
            pthread_mutex_unlock(&queue->mutex);
            return pdFAIL;
        }
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }
#endif

    memcpy(pvBuffer, queue->buffer + (queue->head * queue->item_size), queue->item_size);
    queue->head = (queue->head + 1) % queue->length;
    queue->count--;

#ifdef _WIN32
    WakeConditionVariable(&queue->not_full);
    LeaveCriticalSection(&queue->mutex);
#else
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);
#endif

    return pdPASS;
}

void vQueueDelete(QueueHandle_t xQueue) {
    freertos_queue_t *queue = (freertos_queue_t *)xQueue;
    if (queue == NULL) {
        return;
    }
#ifdef _WIN32
    DeleteCriticalSection(&queue->mutex);
#else
    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);
#endif
    free(queue->buffer);
    free(queue);
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)malloc(sizeof(freertos_semaphore_t));
    if (sem == NULL) {
        return NULL;
    }
#ifdef _WIN32
    InitializeCriticalSection(&sem->mutex);
#else
    pthread_mutex_init(&sem->mutex, NULL);
#endif
    return sem;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xTicksToWait) {
    (void)xTicksToWait;
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return pdFAIL;
    }
#ifdef _WIN32
    EnterCriticalSection(&sem->mutex);
#else
    pthread_mutex_lock(&sem->mutex);
#endif
    return pdPASS;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return pdFAIL;
    }
#ifdef _WIN32
    LeaveCriticalSection(&sem->mutex);
#else
    pthread_mutex_unlock(&sem->mutex);
#endif
    return pdPASS;
}

void vSemaphoreDelete(SemaphoreHandle_t xSemaphore) {
    freertos_semaphore_t *sem = (freertos_semaphore_t *)xSemaphore;
    if (sem == NULL) {
        return;
    }
#ifdef _WIN32
    DeleteCriticalSection(&sem->mutex);
#else
    pthread_mutex_destroy(&sem->mutex);
#endif
    free(sem);
}
