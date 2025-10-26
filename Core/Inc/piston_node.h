#ifndef PISTON_NODE_H
#define PISTON_NODE_H

#include "base_node.h"

// ============================================================================
// ESTRUTURA DO NÓ DO PISTÃO
// ============================================================================

typedef struct {
    BaseNode_t base;  // Herda de BaseNode_t

    // Configuração específica
    Color_t target_color;        // Cor alvo (BLUE ou GREEN)
    char direction[16];          // "esquerda" ou "direita"

    // Estado
    bool is_activated;
    uint64_t last_activation_time_us;

    // Estatísticas
    uint32_t activations_count;
    uint32_t total_objects_processed;
    uint32_t successful_activations;
    uint32_t missed_deadlines;

    // Tarefa de controle
    TaskHandle_t control_task_handle;

    // Fila de ativações agendadas
    QueueHandle_t activation_queue;

} PistonNode_t;

// ============================================================================
// ESTRUTURA PARA ATIVAÇÃO AGENDADA
// ============================================================================

typedef struct {
    ColorDetectionEvent_t detection;
    uint32_t delay_us;
} ScheduledActivation_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o nó do pistão
 *
 * @param piston Ponteiro para o nó do pistão
 * @param node_id ID do nó
 * @param target_color Cor alvo
 * @param direction Direção ("esquerda" ou "direita")
 * @return true se sucesso, false se falha
 */
bool PistonNode_Init(PistonNode_t *piston, const char *node_id,
                     Color_t target_color, const char *direction);

/**
 * @brief Inicia o nó do pistão
 *
 * @param piston Ponteiro para o nó do pistão
 * @return true se sucesso, false se falha
 */
bool PistonNode_Start(PistonNode_t *piston);

/**
 * @brief Para o nó do pistão
 *
 * @param piston Ponteiro para o nó do pistão
 */
void PistonNode_Stop(PistonNode_t *piston);

/**
 * @brief Ativa o pistão fisicamente (controla GPIO)
 *
 * @param piston Ponteiro para o nó do pistão
 */
void PistonNode_PhysicalActivation(PistonNode_t *piston);

/**
 * @brief Obtém estatísticas do pistão
 *
 * @param piston Ponteiro para o nó do pistão
 */
void PistonNode_PrintStats(PistonNode_t *piston);

/**
 * @brief Cria nó Pistão A (VERDE -> DIREITA)
 *
 * @param piston Ponteiro para o nó do pistão
 * @return true se sucesso, false se falha
 */
bool PistonNode_CreatePistonA(PistonNode_t *piston);

/**
 * @brief Cria nó Pistão B (AZUL -> ESQUERDA)
 *
 * @param piston Ponteiro para o nó do pistão
 * @return true se sucesso, false se falha
 */
bool PistonNode_CreatePistonB(PistonNode_t *piston);

#endif // PISTON_NODE_H
