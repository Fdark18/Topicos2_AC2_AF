#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include "base_node.h"

// ============================================================================
// ESTRUTURA DO NÓ DA CÂMERA
// ============================================================================

typedef struct {
    BaseNode_t base;  // Herda de BaseNode_t

    // Estatísticas
    uint32_t total_detections;
    uint32_t blue_detections;
    uint32_t green_detections;

    // Configurações de detecção
    uint32_t detection_threshold_pixels;
    float min_quality;
    uint32_t detection_cooldown_ms;
    uint64_t last_detection_time_us;

    // Controle de objetos únicos
    bool object_in_frame;           // Se há objeto atualmente no quadro
    Color_t current_object_color;   // Cor do objeto atual
    uint32_t object_counter;        // Contador de objetos detectados
    uint64_t object_entry_time_us;  // Momento que objeto entrou

    // Portas de destino (pistões)
    uint16_t piston_ports[2];
    uint8_t piston_count;

    // Tarefa de processamento
    TaskHandle_t processing_task_handle;

} CameraNode_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o nó da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 * @param node_id ID do nó
 * @param piston_ports Array com portas dos pistões
 * @param piston_count Número de pistões
 * @return true se sucesso, false se falha
 */
bool CameraNode_Init(CameraNode_t *camera, const char *node_id,
                     uint16_t *piston_ports, uint8_t piston_count);

/**
 * @brief Inicia o nó da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 * @return true se sucesso, false se falha
 */
bool CameraNode_Start(CameraNode_t *camera);

/**
 * @brief Para o nó da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 */
void CameraNode_Stop(CameraNode_t *camera);

/**
 * @brief Simula detecção de cor (para teste sem câmera real)
 *
 * @param camera Ponteiro para o nó da câmera
 * @param color Cor a simular
 * @param pixels Número de pixels detectados
 * @param quality Qualidade da detecção (0.0 a 1.0)
 */
void CameraNode_SimulateDetection(CameraNode_t *camera, Color_t color,
                                  uint32_t pixels, float quality);

/**
 * @brief Simula saída do objeto do campo da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 */
void CameraNode_SimulateObjectExit(CameraNode_t *camera);

/**
 * @brief Processa frame da câmera e detecta cores
 * (Requer integração com biblioteca de visão computacional)
 *
 * @param camera Ponteiro para o nó da câmera
 * @param frame_data Dados do frame
 * @param frame_size Tamanho do frame
 * @return Color_t Cor detectada
 */
Color_t CameraNode_ProcessFrame(CameraNode_t *camera, uint8_t *frame_data,
                                uint32_t frame_size);

/**
 * @brief Obtém estatísticas do nó da câmera
 *
 * @param camera Ponteiro para o nó da câmera
 */
void CameraNode_PrintStats(CameraNode_t *camera);

#endif // CAMERA_NODE_H
