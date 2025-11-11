#ifndef CONVEYOR_NODE_H
#define CONVEYOR_NODE_H

#include "base_node.h"

// ============================================================================
// ESTRUTURA DO NÓ DA ESTEIRA
// ============================================================================

typedef enum {
    CONVEYOR_STOPPED = 0,
    CONVEYOR_RUNNING,
    CONVEYOR_EMERGENCY_STOP
} ConveyorState_t;

typedef struct {
    BaseNode_t base;  // Herda de BaseNode_t

    // Estado da esteira
    ConveyorState_t state;
    uint32_t speed_mm_s;  // Velocidade em mm/s
    uint64_t start_time_us;
    uint64_t total_runtime_us;

    // Estatísticas
    uint32_t objects_transported;
    uint32_t total_stops;
    uint32_t emergency_stops;

    // Tarefa de controle
    TaskHandle_t control_task_handle;

    // GPIO do motor (simulado)
    uint8_t motor_gpio;
    uint8_t motor_pwm_channel;

} ConveyorNode_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o nó da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @param node_id ID do nó
 * @param motor_gpio GPIO do motor
 * @param speed_mm_s Velocidade da esteira em mm/s
 * @return true se sucesso, false se falha
 */
bool ConveyorNode_Init(ConveyorNode_t *conveyor, const char *node_id,
                      uint8_t motor_gpio, uint32_t speed_mm_s);

/**
 * @brief Inicia o nó da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @return true se sucesso, false se falha
 */
bool ConveyorNode_Start(ConveyorNode_t *conveyor);

/**
 * @brief Para o nó da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 */
void ConveyorNode_Stop(ConveyorNode_t *conveyor);

/**
 * @brief Inicia movimento da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @return true se sucesso, false se falha
 */
bool ConveyorNode_Run(ConveyorNode_t *conveyor);

/**
 * @brief Para movimento da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @param emergency Se true, é uma parada de emergência
 * @return true se sucesso, false se falha
 */
bool ConveyorNode_Halt(ConveyorNode_t *conveyor, bool emergency);

/**
 * @brief Ajusta velocidade da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @param speed_mm_s Nova velocidade em mm/s
 * @return true se sucesso, false se falha
 */
bool ConveyorNode_SetSpeed(ConveyorNode_t *conveyor, uint32_t speed_mm_s);

/**
 * @brief Obtém velocidade atual da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @return Velocidade em mm/s
 */
uint32_t ConveyorNode_GetSpeed(ConveyorNode_t *conveyor);

/**
 * @brief Obtém estado atual da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @return Estado da esteira
 */
ConveyorState_t ConveyorNode_GetState(ConveyorNode_t *conveyor);

/**
 * @brief Notifica esteira sobre transporte de objeto
 *
 * @param conveyor Ponteiro para o nó da esteira
 */
void ConveyorNode_NotifyObjectTransported(ConveyorNode_t *conveyor);

/**
 * @brief Calcula tempo estimado para percorrer distância
 *
 * @param conveyor Ponteiro para o nó da esteira
 * @param distance_mm Distância em mm
 * @return Tempo em microssegundos
 */
uint32_t ConveyorNode_CalculateTravelTime(ConveyorNode_t *conveyor, uint32_t distance_mm);

/**
 * @brief Obtém estatísticas da esteira
 *
 * @param conveyor Ponteiro para o nó da esteira
 */
void ConveyorNode_PrintStats(ConveyorNode_t *conveyor);

#endif // CONVEYOR_NODE_H
