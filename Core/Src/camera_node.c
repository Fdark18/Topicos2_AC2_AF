#include "camera_node.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// TAREFA DE PROCESSAMENTO DE IMAGEM
// ============================================================================

static void CameraNode_ProcessingTask(void *pvParameters) {
    CameraNode_t *camera = (CameraNode_t *)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Processa a cada 100ms

    printf("[%s] Tarefa de processamento iniciada\n", camera->base.node_id);

    while (camera->base.running) {
        // Em um sistema real, aqui seria capturado frame da câmera
        // e processado para detectar cores
        // Por enquanto, apenas aguarda eventos externos via SimulateDetection

        vTaskDelay(xFrequency);
    }

    vTaskDelete(NULL);
}

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

static void CameraNode_SendDetectionEvent(CameraNode_t *camera,
                                         ColorDetectionEvent_t *event) {
    Message_t message;
    message.type = MSG_COLOR_DETECTED;

    // Copia dados do evento para a mensagem
    memcpy(message.data, event, sizeof(ColorDetectionEvent_t));
    message.data_len = sizeof(ColorDetectionEvent_t);

    // Envia para todos os pistões registrados
    for (uint8_t i = 0; i < camera->piston_count; i++) {
        if (BaseNode_SendMessage(&camera->base, &message)) {
            printf("[%s] Evento enviado para pistão na porta %u\n",
                   camera->base.node_id, camera->piston_ports[i]);
        }
    }
}

// ============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// ============================================================================

bool CameraNode_Init(CameraNode_t *camera, const char *node_id,
                     uint16_t *piston_ports, uint8_t piston_count) {
    if (camera == NULL || node_id == NULL) {
        return false;
    }

    // Inicializa nó base
    if (!BaseNode_Init(&camera->base, node_id, NODE_TYPE_CAMERA)) {
        return false;
    }

    // Configurações padrão
    camera->total_detections = 0;
    camera->blue_detections = 0;
    camera->green_detections = 0;

    camera->detection_threshold_pixels = 300;
    camera->min_quality = 0.4f;
    camera->detection_cooldown_ms = 2000;  // 2 segundos entre objetos
    camera->last_detection_time_us = 0;

    // Controle de objetos únicos
    camera->object_in_frame = false;
    camera->current_object_color = COLOR_UNKNOWN;
    camera->object_counter = 0;
    camera->object_entry_time_us = 0;

    // Copia portas dos pistões
    camera->piston_count = (piston_count > 2) ? 2 : piston_count;
    if (piston_ports != NULL) {
        for (uint8_t i = 0; i < camera->piston_count; i++) {
            camera->piston_ports[i] = piston_ports[i];
        }
    }

    printf("[%s] Nó da câmera inicializado\n", camera->base.node_id);
    return true;
}

bool CameraNode_Start(CameraNode_t *camera) {
    if (camera == NULL) {
        return false;
    }

    // Inicia nó base
    if (!BaseNode_Start(&camera->base)) {
        return false;
    }

    // Cria tarefa de processamento
    BaseTaskCreate(CameraNode_ProcessingTask, "CamProcess", 4096, camera, 4,
                   &camera->processing_task_handle);

    if (camera->processing_task_handle == NULL) {
        printf("[%s] ERRO: Falha ao criar tarefa de processamento\n",
               camera->base.node_id);
        BaseNode_Stop(&camera->base);
        return false;
    }

    printf("[%s] Nó da câmera iniciado\n", camera->base.node_id);
    return true;
}

void CameraNode_Stop(CameraNode_t *camera) {
    if (camera == NULL) {
        return;
    }

    BaseNode_Stop(&camera->base);
    printf("[%s] Nó da câmera parado\n", camera->base.node_id);
}

void CameraNode_SimulateDetection(CameraNode_t *camera, Color_t color,
                                  uint32_t pixels, float quality) {
    if (camera == NULL || color == COLOR_UNKNOWN) {
        return;
    }

    uint64_t current_time_us = BaseNode_GetTimestampUs();

    // Valida qualidade e número de pixels primeiro
    if (pixels < camera->detection_threshold_pixels || quality < camera->min_quality) {
        printf("[%s] REJEITADO: Qualidade insuficiente (pixels=%u, quality=%.2f)\n",
               camera->base.node_id, pixels, quality);
        return;
    }

    // Lógica de detecção única: detecta apenas quando objeto ENTRA na câmera
    if (!camera->object_in_frame) {
        // Objeto entrando no campo de visão
        camera->object_in_frame = true;
        camera->current_object_color = color;
        camera->object_entry_time_us = current_time_us;
        camera->object_counter++;

        // Cria evento de detecção
        ColorDetectionEvent_t event;
        event.timestamp_us = current_time_us;
        event.color = color;
        event.confidence = quality;
        event.pixels = pixels;
        event.quality = quality;
        strncpy(event.camera_id, camera->base.node_id, MAX_NODE_ID_LEN - 1);

        // Atualiza estatísticas
        camera->total_detections++;
        if (color == COLOR_BLUE) {
            camera->blue_detections++;
        } else if (color == COLOR_GREEN) {
            camera->green_detections++;
        }

        camera->last_detection_time_us = current_time_us;

        // Log da detecção
        const char *piston_info = (color == COLOR_GREEN) ?
            "PISTAO A (VERDE -> DIREITA)" : "PISTAO B (AZUL -> ESQUERDA)";

        printf("\n[%s] ========================================\n", camera->base.node_id);
        printf("[%s] ✓ OBJETO #%u DETECTADO: Cor %s ENTROU no campo\n",
               camera->base.node_id, camera->object_counter,
               BaseNode_ColorToString(color));
        printf("[%s]   Pixels: %u | Qualidade: %.1f%%\n",
               camera->base.node_id, pixels, quality * 100.0f);
        printf("[%s] >>> ACIONANDO: %s sera ativado em 2000ms\n",
               camera->base.node_id, piston_info);
        printf("[%s] ========================================\n\n", camera->base.node_id);

        // Envia evento para pistões
        CameraNode_SendDetectionEvent(camera, &event);
    } else {
        // Objeto ainda está no campo - não detecta novamente
        printf("[%s] Objeto #%u (%s) ainda no campo - aguardando saída...\n",
               camera->base.node_id, camera->object_counter,
               BaseNode_ColorToString(camera->current_object_color));
    }
}

/**
 * @brief Simula saída do objeto do campo da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 */
void CameraNode_SimulateObjectExit(CameraNode_t *camera) {
    if (camera == NULL || !camera->object_in_frame) {
        return;
    }

    uint64_t current_time_us = BaseNode_GetTimestampUs();
    uint64_t time_in_frame = current_time_us - camera->object_entry_time_us;

    printf("[%s] << Objeto #%u (%s) SAIU do campo após %llu ms\n",
           camera->base.node_id, camera->object_counter,
           BaseNode_ColorToString(camera->current_object_color),
           (unsigned long long)(time_in_frame / 1000));

    // Libera para próxima detecção
    camera->object_in_frame = false;
    camera->current_object_color = COLOR_UNKNOWN;
}

Color_t CameraNode_ProcessFrame(CameraNode_t *camera, uint8_t *frame_data,
                                uint32_t frame_size) {
    // Esta função requer integração com biblioteca de processamento de imagem
    // Para ESP32-CAM, seria usado ESP-WHO ou OpenCV-ESP32

    // Exemplo de lógica (simplificada):
    // 1. Converter para HSV
    // 2. Aplicar máscaras de cor
    // 3. Contar pixels de cada cor
    // 4. Validar qualidade
    // 5. Retornar cor predominante

    // Por enquanto, retorna desconhecido
    return COLOR_UNKNOWN;
}

void CameraNode_PrintStats(CameraNode_t *camera) {
    if (camera == NULL) {
        return;
    }

    float avg_qos = BaseNode_GetAverageQoS(&camera->base);

    printf("\n[%s] ===== ESTATISTICAS DA CAMERA =====\n", camera->base.node_id);
    printf("[%s] Total de detecções: %u\n", camera->base.node_id,
           camera->total_detections);
    printf("[%s] Detecções AZUL: %u\n", camera->base.node_id,
           camera->blue_detections);
    printf("[%s] Detecções VERDE: %u\n", camera->base.node_id,
           camera->green_detections);
    printf("[%s] QoS médio: %.1f%%\n", camera->base.node_id, avg_qos * 100.0f);
    printf("[%s] ====================================\n\n", camera->base.node_id);
}
