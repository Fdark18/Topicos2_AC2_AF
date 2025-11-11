#ifndef SAFETY_NODE_H
#define SAFETY_NODE_H

#include "base_node.h"

// ============================================================================
// ESTRUTURA DO NÓ DE SEGURANÇA
// ============================================================================

typedef struct {
    BaseNode_t base;  // Herda de BaseNode_t

    // Estado de emergência
    bool emergency_stop_active;
    uint64_t emergency_timestamp_us;

    // Estatísticas
    uint32_t emergency_activations;
    uint32_t total_stops_issued;

    // Tarefas
    TaskHandle_t monitor_task_handle;

    // GPIO do botão de emergência (simulado)
    uint8_t emergency_button_gpio;
    bool button_pressed;

    // Callback para notificar outros nós
    void (*on_emergency_callback)(void);

} SafetyNode_t;

// ============================================================================
// FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * @brief Inicializa o nó de segurança
 *
 * @param safety Ponteiro para o nó de segurança
 * @param node_id ID do nó
 * @param emergency_gpio GPIO do botão de emergência
 * @return true se sucesso, false se falha
 */
bool SafetyNode_Init(SafetyNode_t *safety, const char *node_id, uint8_t emergency_gpio);

/**
 * @brief Inicia o nó de segurança
 *
 * @param safety Ponteiro para o nó de segurança
 * @return true se sucesso, false se falha
 */
bool SafetyNode_Start(SafetyNode_t *safety);

/**
 * @brief Para o nó de segurança
 *
 * @param safety Ponteiro para o nó de segurança
 */
void SafetyNode_Stop(SafetyNode_t *safety);

/**
 * @brief Simula acionamento do botão de emergência
 *
 * @param safety Ponteiro para o nó de segurança
 */
void SafetyNode_TriggerEmergency(SafetyNode_t *safety);

/**
 * @brief Reseta estado de emergência (após correção)
 *
 * @param safety Ponteiro para o nó de segurança
 * @return true se sucesso, false se falha
 */
bool SafetyNode_ResetEmergency(SafetyNode_t *safety);

/**
 * @brief Verifica se sistema está em estado de emergência
 *
 * @param safety Ponteiro para o nó de segurança
 * @return true se em emergência, false caso contrário
 */
bool SafetyNode_IsEmergencyActive(SafetyNode_t *safety);

/**
 * @brief Registra callback para notificar emergência
 *
 * @param safety Ponteiro para o nó de segurança
 * @param callback Função callback
 */
void SafetyNode_RegisterEmergencyCallback(SafetyNode_t *safety, void (*callback)(void));

/**
 * @brief Obtém estatísticas do nó de segurança
 *
 * @param safety Ponteiro para o nó de segurança
 */
void SafetyNode_PrintStats(SafetyNode_t *safety);

#endif // SAFETY_NODE_H
