#include "base_node.h"
#include "timing_config.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// TAREFAS FREERTOS INTERNAS
// ============================================================================

static void BaseNode_ListenTask(void *pvParameters) {
    BaseNode_t *node = (BaseNode_t *)pvParameters;
    Message_t message;

    while (node->running) {
        // Aguarda mensagens na fila de recepção
        if (xQueueReceive(node->rx_queue, &message, portMAX_DELAY) == pdTRUE) {
            // Chama handler customizado se registrado
            if (node->message_handler != NULL) {
                node->message_handler(node, &message);
            }
        }
    }

    vTaskDelete(NULL);
}

static void BaseNode_HeartbeatTask(void *pvParameters) {
    BaseNode_t *node = (BaseNode_t *)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 segundo

    while (node->running) {
        node->last_heartbeat = BaseNode_GetTimestampUs();
        vTaskDelay(xFrequency);
    }

    vTaskDelete(NULL);
}

// ============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// ============================================================================

bool BaseNode_Init(BaseNode_t *node, const char *node_id, NodeType_t node_type) {
    if (node == NULL || node_id == NULL) {
        return false;
    }

    // Inicializa campos básicos
    strncpy(node->node_id, node_id, MAX_NODE_ID_LEN - 1);
    node->node_id[MAX_NODE_ID_LEN - 1] = '\0';
    node->node_type = node_type;
    node->running = false;
    node->last_heartbeat = 0;

    // Configurações padrão de timing
    node->deadline_us = END_TO_END_DEADLINE_US;
    node->belt_speed_mm_s = BELT_SPEED_MM_S;
    node->camera_to_piston_distance_mm = CAMERA_TO_PISTON_DISTANCE_MM;

    // Inicializa métricas
    node->metrics_count = 0;
    node->metrics_index = 0;
    memset(node->metrics_history, 0, sizeof(node->metrics_history));

    // Handler padrão é NULL
    node->message_handler = NULL;

    // Cria filas de comunicação
    node->rx_queue = xQueueCreate(QUEUE_SIZE, sizeof(Message_t));
    node->tx_queue = xQueueCreate(QUEUE_SIZE, sizeof(Message_t));

    if (node->rx_queue == NULL || node->tx_queue == NULL) {
        printf("[%s] ERRO: Falha ao criar filas\n", node->node_id);
        return false;
    }

    // Cria mutex para métricas
    node->metrics_mutex = xSemaphoreCreateMutex();
    if (node->metrics_mutex == NULL) {
        printf("[%s] ERRO: Falha ao criar mutex\n", node->node_id);
        return false;
    }

    printf("[%s] Nó inicializado (Tipo: %s)\n",
           node->node_id, BaseNode_TypeToString(node->node_type));

    return true;
}

bool BaseNode_Start(BaseNode_t *node) {
    if (node == NULL || node->running) {
        return false;
    }

    node->running = true;

    // Cria tarefa de escuta de mensagens
    BaseTaskCreate(BaseNode_ListenTask, "Listen", 2048, node, 3,
                   &node->listen_task_handle);

    // Cria tarefa de heartbeat
    BaseTaskCreate(BaseNode_HeartbeatTask, "Heartbeat", 1024, node, 1,
                   &node->heartbeat_task_handle);

    if (node->listen_task_handle == NULL || node->heartbeat_task_handle == NULL) {
        printf("[%s] ERRO: Falha ao criar tarefas\n", node->node_id);
        node->running = false;
        return false;
    }

    printf("[%s] Nó iniciado\n", node->node_id);
    return true;
}

void BaseNode_Stop(BaseNode_t *node) {
    if (node == NULL) {
        return;
    }

    node->running = false;

    // Aguarda tarefas terminarem
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("[%s] Nó parado\n", node->node_id);
}

bool BaseNode_SendMessage(BaseNode_t *node, Message_t *message) {
    if (node == NULL || message == NULL) {
        return false;
    }

    // Adiciona informações do sender
    message->timestamp_us = BaseNode_GetTimestampUs();
    strncpy(message->sender_id, node->node_id, MAX_NODE_ID_LEN - 1);
    message->sender_type = node->node_type;

    // Envia para fila de transmissão
    if (xQueueSend(node->tx_queue, message, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("[%s] ERRO: Fila TX cheia\n", node->node_id);
        return false;
    }

    return true;
}

void BaseNode_RegisterMessageHandler(BaseNode_t *node,
                                     void (*handler)(BaseNode_t*, Message_t*)) {
    if (node != NULL) {
        node->message_handler = handler;
    }
}

uint32_t BaseNode_CalculateBeltDelay(BaseNode_t *node) {
    if (node == NULL || node->belt_speed_mm_s == 0) {
        return 0;
    }

    // Tempo = Distância / Velocidade
    float delay_seconds = (float)node->camera_to_piston_distance_mm /
                         (float)node->belt_speed_mm_s;

    return (uint32_t)(delay_seconds * 1000000.0f); // Converte para microssegundos
}

uint64_t BaseNode_GetTimestampUs(void) {
    // Em ESP32, usa esp_timer_get_time()
    // Aqui simulamos com ticks do FreeRTOS
    TickType_t ticks = xTaskGetTickCount();
    return (uint64_t)ticks * 1000; // Converte para microssegundos (assumindo 1ms tick)
}

RealTimeMetrics_t BaseNode_CalculateMetrics(BaseNode_t *node,
                                            uint64_t detection_timestamp_us,
                                            uint64_t activation_timestamp_us) {
    RealTimeMetrics_t metrics = {0};

    if (node == NULL) {
        return metrics;
    }

    // Calcula latência total
    metrics.detection_to_activation_latency_us =
        (uint32_t)(activation_timestamp_us - detection_timestamp_us);

    metrics.deadline_us = node->deadline_us;
    metrics.deadline_met = (metrics.detection_to_activation_latency_us <= metrics.deadline_us);

    // Calcula jitter baseado no histórico
    if (xSemaphoreTake(node->metrics_mutex, portMAX_DELAY) == pdTRUE) {
        if (node->metrics_count > 0) {
            uint8_t last_idx = (node->metrics_index == 0) ?
                              99 : node->metrics_index - 1;
            uint32_t last_latency = node->metrics_history[last_idx]
                                   .detection_to_activation_latency_us;

            metrics.jitter_us = (metrics.detection_to_activation_latency_us > last_latency) ?
                               (metrics.detection_to_activation_latency_us - last_latency) :
                               (last_latency - metrics.detection_to_activation_latency_us);
        }
        xSemaphoreGive(node->metrics_mutex);
    }

    // Calcula QoS score
    if (metrics.deadline_us == 0) {
        metrics.deadline_us = 1;
    }

    float latency_ratio = (float)metrics.detection_to_activation_latency_us /
                         (float)metrics.deadline_us;
    metrics.qos_score = 1.0f - latency_ratio;
    if (metrics.qos_score < 0.0f) {
        metrics.qos_score = 0.0f;
    }

    if (!metrics.deadline_met) {
        metrics.qos_score *= 0.5f; // Penaliza se deadline não foi cumprido
    }

    return metrics;
}

void BaseNode_AddMetric(BaseNode_t *node, RealTimeMetrics_t *metrics) {
    if (node == NULL || metrics == NULL) {
        return;
    }

    if (xSemaphoreTake(node->metrics_mutex, portMAX_DELAY) == pdTRUE) {
        // Adiciona no buffer circular
        node->metrics_history[node->metrics_index] = *metrics;
        node->metrics_index = (node->metrics_index + 1) % 100;

        if (node->metrics_count < 100) {
            node->metrics_count++;
        }

        xSemaphoreGive(node->metrics_mutex);
    }
}

float BaseNode_GetAverageQoS(BaseNode_t *node) {
    if (node == NULL) {
        return 0.0f;
    }

    float sum = 0.0f;
    uint8_t count = 0;

    if (xSemaphoreTake(node->metrics_mutex, portMAX_DELAY) == pdTRUE) {
        for (uint8_t i = 0; i < node->metrics_count; i++) {
            sum += node->metrics_history[i].qos_score;
            count++;
        }
        xSemaphoreGive(node->metrics_mutex);
    }

    return (count > 0) ? (sum / count) : 0.0f;
}

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

const char* BaseNode_TypeToString(NodeType_t type) {
    switch (type) {
        case NODE_TYPE_CAMERA:     return "CAMERA";
        case NODE_TYPE_PISTON_A:   return "PISTON_A";
        case NODE_TYPE_PISTON_B:   return "PISTON_B";
        case NODE_TYPE_MONITOR:    return "MONITOR";
        default:                   return "UNKNOWN";
    }
}

const char* BaseNode_ColorToString(Color_t color) {
    switch (color) {
        case COLOR_BLUE:    return "AZUL";
        case COLOR_GREEN:   return "VERDE";
        case COLOR_UNKNOWN:
        default:            return "DESCONHECIDO";
    }
}

Color_t BaseNode_StringToColor(const char *str) {
    if (str == NULL) {
        return COLOR_UNKNOWN;
    }

    if (strcmp(str, "azul") == 0 || strcmp(str, "AZUL") == 0) {
        return COLOR_BLUE;
    } else if (strcmp(str, "verde") == 0 || strcmp(str, "VERDE") == 0) {
        return COLOR_GREEN;
    }

    return COLOR_UNKNOWN;
}
