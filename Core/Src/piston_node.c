#include "piston_node.h"
#include "timing_config.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// TAREFAS FREERTOS
// ============================================================================

static void PistonNode_ControlTask(void *pvParameters) {
    PistonNode_t *piston = (PistonNode_t *)pvParameters;
    ScheduledActivation_t scheduled;

    printf("[%s] Tarefa de controle iniciada (Cor: %s, Direção: %s)\n",
           piston->base.node_id,
           BaseNode_ColorToString(piston->target_color),
           piston->direction);

    while (piston->base.running) {
        // Aguarda ativações agendadas
        if (xQueueReceive(piston->activation_queue, &scheduled, portMAX_DELAY) == pdTRUE) {
            // Aguarda o delay calculado
            uint32_t delay_ms = scheduled.delay_us / 1000;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));

            // Timestamp da ativação
            uint64_t activation_timestamp_us = BaseNode_GetTimestampUs();

            // Ativa pistão fisicamente
            piston->is_activated = true;
            piston->last_activation_time_us = activation_timestamp_us;
            piston->activations_count++;

            printf("\n[%s] *** PISTAO EXECUTANDO MOVIMENTO ***\n",
                   piston->base.node_id);
            printf("[%s]   Objeto: %s | Direção: %s | Delay: %u ms\n",
                   piston->base.node_id,
                   BaseNode_ColorToString(scheduled.detection.color),
                   piston->direction, delay_ms);

            // Simula movimento físico
            PistonNode_PhysicalActivation(piston);

            // Calcula métricas
            RealTimeMetrics_t metrics = BaseNode_CalculateMetrics(
                &piston->base,
                scheduled.detection.timestamp_us,
                activation_timestamp_us
            );

            // Adiciona ao histórico
            BaseNode_AddMetric(&piston->base, &metrics);

            // Atualiza estatísticas
            piston->total_objects_processed++;
            if (metrics.deadline_met) {
                piston->successful_activations++;
            } else {
                piston->missed_deadlines++;
            }

            // Log de métricas
            printf("[%s] METRICAS DE TEMPO REAL:\n", piston->base.node_id);
            printf("[%s]   - Latencia total: %u ms\n",
                   piston->base.node_id,
                   metrics.detection_to_activation_latency_us / 1000);
            printf("[%s]   - Deadline: %u ms [%s]\n",
                   piston->base.node_id,
                   metrics.deadline_us / 1000,
                   metrics.deadline_met ? "OK CUMPRIDO" : "ERRO PERDIDO");
            printf("[%s]   - Jitter: %u ms\n",
                   piston->base.node_id, metrics.jitter_us / 1000);
            printf("[%s]   - QoS Score: %.1f%%\n",
                   piston->base.node_id, metrics.qos_score * 100.0f);
            printf("[%s]   - Qualidade detecção: %.1f%%\n",
                   piston->base.node_id, scheduled.detection.quality * 100.0f);
            printf("[%s] <<< PISTAO CONCLUIDO <<<\n\n", piston->base.node_id);

            piston->is_activated = false;
        }
    }

    vTaskDelete(NULL);
}

// ============================================================================
// HANDLER DE MENSAGENS
// ============================================================================

static void PistonNode_MessageHandler(BaseNode_t *base_node, Message_t *message) {
    PistonNode_t *piston = (PistonNode_t *)base_node;

    if (message->type == MSG_COLOR_DETECTED) {
        // Extrai evento de detecção
        ColorDetectionEvent_t *detection = (ColorDetectionEvent_t *)message->data;

        // Verifica se é a cor alvo deste pistão
        if (detection->color == piston->target_color) {
            printf("[%s] >> RECEBIDO: Detecção de %s - PISTAO %s (%s) será acionado\n",
                   piston->base.node_id,
                   BaseNode_ColorToString(detection->color),
                   piston->base.node_id,
                   piston->direction);

            // Calcula delay baseado na esteira
            uint32_t belt_delay_us = BaseNode_CalculateBeltDelay(&piston->base);

            printf("[%s] >>> AGENDANDO: Pistão será ativado em %u ms para %s -> %s\n",
                   piston->base.node_id,
                   belt_delay_us / 1000,
                   BaseNode_ColorToString(detection->color),
                   piston->direction);

            // Agenda ativação
            ScheduledActivation_t scheduled;
            scheduled.detection = *detection;
            scheduled.delay_us = belt_delay_us;

            if (xQueueSend(piston->activation_queue, &scheduled, 0) != pdTRUE) {
                printf("[%s] ERRO: Fila de ativações cheia\n",
                       piston->base.node_id);
            }
        } else {
            // Ignora silenciosamente cores que não são alvo
        }
    }
}

// ============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// ============================================================================

bool PistonNode_Init(PistonNode_t *piston, const char *node_id,
                     Color_t target_color, const char *direction) {
    if (piston == NULL || node_id == NULL || direction == NULL) {
        return false;
    }

    // Determina tipo do nó baseado na cor
    NodeType_t node_type = (target_color == COLOR_GREEN) ?
                          NODE_TYPE_PISTON_A : NODE_TYPE_PISTON_B;

    // Inicializa nó base
    if (!BaseNode_Init(&piston->base, node_id, node_type)) {
        return false;
    }

    // Ajusta parâmetros de timing com base na configuração física
    piston->base.belt_speed_mm_s = BELT_SPEED_MM_S;
    piston->base.camera_to_piston_distance_mm = CAMERA_TO_PISTON_DISTANCE_MM;
    uint32_t travel_time_us = BaseNode_CalculateBeltDelay(&piston->base);
    piston->base.deadline_us = travel_time_us + SAFETY_MARGIN_US + PISTON_ACTIVATION_TIME_US;

    // Configurações específicas
    piston->target_color = target_color;
    strncpy(piston->direction, direction, sizeof(piston->direction) - 1);
    piston->direction[sizeof(piston->direction) - 1] = '\0';

    // Estado inicial
    piston->is_activated = false;
    piston->last_activation_time_us = 0;

    // Estatísticas
    piston->activations_count = 0;
    piston->total_objects_processed = 0;
    piston->successful_activations = 0;
    piston->missed_deadlines = 0;

    // Cria fila de ativações
    piston->activation_queue = xQueueCreate(5, sizeof(ScheduledActivation_t));
    if (piston->activation_queue == NULL) {
        printf("[%s] ERRO: Falha ao criar fila de ativações\n", node_id);
        return false;
    }

    // Registra handler de mensagens
    BaseNode_RegisterMessageHandler(&piston->base, PistonNode_MessageHandler);

    printf("[%s] Pistão inicializado (Cor: %s, Direção: %s)\n",
           piston->base.node_id,
           BaseNode_ColorToString(piston->target_color),
           piston->direction);

    return true;
}

bool PistonNode_Start(PistonNode_t *piston) {
    if (piston == NULL) {
        return false;
    }

    // Inicia nó base
    if (!BaseNode_Start(&piston->base)) {
        return false;
    }

    // Cria tarefa de controle
    BaseTaskCreate(PistonNode_ControlTask, "PistonCtrl", 4096, piston, 5,
                   &piston->control_task_handle);

    if (piston->control_task_handle == NULL) {
        printf("[%s] ERRO: Falha ao criar tarefa de controle\n",
               piston->base.node_id);
        BaseNode_Stop(&piston->base);
        return false;
    }

    printf("[%s] Pistão iniciado - Aguardando detecções de %s...\n",
           piston->base.node_id,
           BaseNode_ColorToString(piston->target_color));

    return true;
}

void PistonNode_Stop(PistonNode_t *piston) {
    if (piston == NULL) {
        return;
    }

    BaseNode_Stop(&piston->base);
    printf("[%s] Pistão parado\n", piston->base.node_id);
}

void PistonNode_PhysicalActivation(PistonNode_t *piston) {
    // Em hardware real, aqui seria controlado GPIO para acionar servo/solenoide
    // Exemplo para ESP32:
    // gpio_set_level(PISTON_GPIO_PIN, 1);  // Ativa
    // vTaskDelay(pdMS_TO_TICKS(50));       // Mantém ativado
    // gpio_set_level(PISTON_GPIO_PIN, 0);  // Desativa

    // Simula tempo de movimento físico (50ms)
    vTaskDelay(pdMS_TO_TICKS(50));
}

void PistonNode_PrintStats(PistonNode_t *piston) {
    if (piston == NULL) {
        return;
    }

    float success_rate = 0.0f;
    if (piston->total_objects_processed > 0) {
        success_rate = (float)piston->successful_activations /
                      (float)piston->total_objects_processed;
    }

    float avg_qos = BaseNode_GetAverageQoS(&piston->base);

    printf("\n[%s] ===== ESTATISTICAS DO PISTAO =====\n",
           piston->base.node_id);
    printf("[%s] Cor alvo: %s | Direção: %s\n",
           piston->base.node_id,
           BaseNode_ColorToString(piston->target_color),
           piston->direction);
    printf("[%s] Total de ativações: %u\n",
           piston->base.node_id, piston->activations_count);
    printf("[%s] Objetos processados: %u\n",
           piston->base.node_id, piston->total_objects_processed);
    printf("[%s] Ativações bem-sucedidas: %u\n",
           piston->base.node_id, piston->successful_activations);
    printf("[%s] Deadlines perdidos: %u\n",
           piston->base.node_id, piston->missed_deadlines);
    printf("[%s] Taxa de sucesso: %.1f%%\n",
           piston->base.node_id, success_rate * 100.0f);
    printf("[%s] QoS médio: %.1f%%\n",
           piston->base.node_id, avg_qos * 100.0f);
    printf("[%s] ====================================\n\n",
           piston->base.node_id);
}

bool PistonNode_CreatePistonA(PistonNode_t *piston) {
    return PistonNode_Init(piston, "piston_a", COLOR_GREEN, "direita");
}

bool PistonNode_CreatePistonB(PistonNode_t *piston) {
    return PistonNode_Init(piston, "piston_b", COLOR_BLUE, "esquerda");
}
