#ifndef BASE_NODE_H
#define BASE_NODE_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// DEFINIÇÕES DE TIPOS E CONSTANTES
// ============================================================================

#define MAX_NODE_ID_LEN 32
#define MAX_MESSAGE_SIZE 512
#define QUEUE_SIZE 10
#define DEADLINE_US 50000  // 50ms deadline padrão

// Tipos de nós no sistema distribuído
typedef enum {
    NODE_TYPE_CAMERA,
    NODE_TYPE_PISTON_A,
    NODE_TYPE_PISTON_B,
    NODE_TYPE_MONITOR
} NodeType_t;

// Tipos de mensagem para comunicação
typedef enum {
    MSG_COLOR_DETECTED,
    MSG_PISTON_ACTIVATED,
    MSG_HEARTBEAT,
    MSG_STATUS_REQUEST,
    MSG_STATUS_RESPONSE
} MessageType_t;

// Cores detectáveis
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_BLUE,
    COLOR_GREEN
} Color_t;

// ============================================================================
// ESTRUTURAS DE EVENTOS
// ============================================================================

// Evento de detecção de cor
typedef struct {
    uint64_t timestamp_us;
    Color_t color;
    float confidence;
    uint32_t pixels;
    float quality;
    char camera_id[MAX_NODE_ID_LEN];
} ColorDetectionEvent_t;

// Evento de ativação do pistão
typedef struct {
    uint64_t timestamp_us;
    char piston_id[MAX_NODE_ID_LEN];
    char direction[16];  // "esquerda" ou "direita"
    uint32_t activation_delay_us;
    uint64_t original_detection_timestamp_us;
} PistonActivationEvent_t;

// Métricas de tempo real
typedef struct {
    uint32_t detection_to_activation_latency_us;
    uint32_t deadline_us;
    bool deadline_met;
    uint32_t jitter_us;
    float qos_score;
} RealTimeMetrics_t;

// ============================================================================
// ESTRUTURA DE MENSAGEM GENÉRICA
// ============================================================================

typedef struct {
    MessageType_t type;
    uint64_t timestamp_us;
    char sender_id[MAX_NODE_ID_LEN];
    NodeType_t sender_type;
    uint8_t data[MAX_MESSAGE_SIZE];
    uint16_t data_len;
} Message_t;

// ============================================================================
// ESTRUTURA DO NÓ BASE
// ============================================================================

typedef struct BaseNode {
    // Identificação
    char node_id[MAX_NODE_ID_LEN];
    NodeType_t node_type;

    // Estado
    bool running;
    uint64_t last_heartbeat;

    // Tarefas FreeRTOS
    TaskHandle_t listen_task_handle;
    TaskHandle_t heartbeat_task_handle;
    TaskHandle_t process_task_handle;

    // Comunicação
    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
    SemaphoreHandle_t metrics_mutex;

    // Configurações de timing (em microssegundos)
    uint32_t deadline_us;
    uint32_t belt_speed_mm_s;
    uint32_t camera_to_piston_distance_mm;

    // Métricas
    RealTimeMetrics_t metrics_history[100];
    uint8_t metrics_count;
    uint8_t metrics_index;

    // Handler customizado para mensagens (callback)
    void (*message_handler)(struct BaseNode *node, Message_t *message);

} BaseNode_t;

// ============================================================================
// FUNÇÕES PÚBLICAS DA BASE NODE
// ============================================================================

/**
 * @brief Inicializa um nó base
 *
 * @param node Ponteiro para o nó
 * @param node_id ID do nó
 * @param node_type Tipo do nó
 * @return true se sucesso, false se falha
 */
bool BaseNode_Init(BaseNode_t *node, const char *node_id, NodeType_t node_type);

/**
 * @brief Inicia o nó (cria tarefas FreeRTOS)
 *
 * @param node Ponteiro para o nó
 * @return true se sucesso, false se falha
 */
bool BaseNode_Start(BaseNode_t *node);

/**
 * @brief Para o nó
 *
 * @param node Ponteiro para o nó
 */
void BaseNode_Stop(BaseNode_t *node);

/**
 * @brief Envia mensagem para a fila de transmissão
 *
 * @param node Ponteiro para o nó
 * @param message Mensagem a enviar
 * @return true se sucesso, false se falha
 */
bool BaseNode_SendMessage(BaseNode_t *node, Message_t *message);

/**
 * @brief Registra handler customizado para mensagens
 *
 * @param node Ponteiro para o nó
 * @param handler Função handler
 */
void BaseNode_RegisterMessageHandler(BaseNode_t *node,
                                     void (*handler)(BaseNode_t*, Message_t*));

/**
 * @brief Calcula delay baseado na velocidade da esteira
 *
 * @param node Ponteiro para o nó
 * @return Delay em microssegundos
 */
uint32_t BaseNode_CalculateBeltDelay(BaseNode_t *node);

/**
 * @brief Obtém timestamp atual em microssegundos
 *
 * @return Timestamp em microssegundos
 */
uint64_t BaseNode_GetTimestampUs(void);

/**
 * @brief Calcula métricas de tempo real
 *
 * @param node Ponteiro para o nó
 * @param detection_timestamp_us Timestamp da detecção
 * @param activation_timestamp_us Timestamp da ativação
 * @return Métricas calculadas
 */
RealTimeMetrics_t BaseNode_CalculateMetrics(BaseNode_t *node,
                                            uint64_t detection_timestamp_us,
                                            uint64_t activation_timestamp_us);

/**
 * @brief Adiciona métrica ao histórico
 *
 * @param node Ponteiro para o nó
 * @param metrics Métrica a adicionar
 */
void BaseNode_AddMetric(BaseNode_t *node, RealTimeMetrics_t *metrics);

/**
 * @brief Obtém QoS médio do histórico
 *
 * @param node Ponteiro para o nó
 * @return QoS médio (0.0 a 1.0)
 */
float BaseNode_GetAverageQoS(BaseNode_t *node);

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

/**
 * @brief Converte tipo de nó para string
 *
 * @param type Tipo do nó
 * @return String com o nome do tipo
 */
const char* BaseNode_TypeToString(NodeType_t type);

/**
 * @brief Converte cor para string
 *
 * @param color Cor
 * @return String com o nome da cor
 */
const char* BaseNode_ColorToString(Color_t color);

/**
 * @brief Converte string para cor
 *
 * @param str String
 * @return Cor correspondente
 */
Color_t BaseNode_StringToColor(const char *str);

/**
 * @brief Cria uma tarefa FreeRTOS com parâmetros simplificados
 *
 * @param pvTaskCode Função da tarefa
 * @param pcName Nome da tarefa
 * @param usStackDepth Tamanho da pilha
 * @param pvParameters Parâmetros da tarefa
 * @param uxPriority Prioridade da tarefa
 * @param pxCreatedTask Handle da tarefa criada
 * @return true se sucesso, false se falha
 */
static inline BaseType_t BaseTaskCreate(
    TaskFunction_t pvTaskCode,
    const char *pcName,
    uint16_t usStackDepth,
    void *pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t *pxCreatedTask)
{
    return xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask);
}

#endif // BASE_NODE_H
