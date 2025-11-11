#include "safety_node.h"
#include <string.h>
#include <stdio.h>

// Variável global para estado de emergência (acessível por todos os nós)
static volatile bool g_emergency_stop = false;

// ============================================================================
// TAREFA DE MONITORAMENTO
// ============================================================================

static void SafetyNode_MonitorTask(void *pvParameters) {
    SafetyNode_t *safety = (SafetyNode_t *)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Monitora a cada 10ms (alta frequência)

    printf("[%s] Tarefa de monitoramento de segurança iniciada (prioridade MÁXIMA)\n",
           safety->base.node_id);

    while (safety->base.running) {
        // Verifica botão de emergência (simulado por flag)
        if (safety->button_pressed && !safety->emergency_stop_active) {
            SafetyNode_TriggerEmergency(safety);
        }

        vTaskDelay(xFrequency);
    }

    vTaskDelete(NULL);
}

// ============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// ============================================================================

bool SafetyNode_Init(SafetyNode_t *safety, const char *node_id, uint8_t emergency_gpio) {
    if (safety == NULL || node_id == NULL) {
        return false;
    }

    // Inicializa nó base
    if (!BaseNode_Init(&safety->base, node_id, NODE_TYPE_MONITOR)) {
        return false;
    }

    // Estado inicial
    safety->emergency_stop_active = false;
    safety->emergency_timestamp_us = 0;
    safety->emergency_activations = 0;
    safety->total_stops_issued = 0;
    safety->emergency_button_gpio = emergency_gpio;
    safety->button_pressed = false;
    safety->on_emergency_callback = NULL;

    g_emergency_stop = false;

    printf("[%s] Nó de segurança inicializado (GPIO: %u)\n",
           safety->base.node_id, emergency_gpio);
    printf("[%s] ⚠️  SISTEMA DE PARADA DE EMERGÊNCIA ATIVO ⚠️\n",
           safety->base.node_id);

    return true;
}

bool SafetyNode_Start(SafetyNode_t *safety) {
    if (safety == NULL) {
        return false;
    }

    // Inicia nó base
    if (!BaseNode_Start(&safety->base)) {
        return false;
    }

    // Cria tarefa de monitoramento com MÁXIMA prioridade
    // Prioridade 10 é a mais alta do sistema
    BaseTaskCreate(SafetyNode_MonitorTask, "SafetyMon", 2048, safety, 10,
                   &safety->monitor_task_handle);

    if (safety->monitor_task_handle == NULL) {
        printf("[%s] ERRO: Falha ao criar tarefa de monitoramento\n",
               safety->base.node_id);
        BaseNode_Stop(&safety->base);
        return false;
    }

    printf("[%s] Nó de segurança iniciado com PRIORIDADE MÁXIMA (10)\n",
           safety->base.node_id);

    return true;
}

void SafetyNode_Stop(SafetyNode_t *safety) {
    if (safety == NULL) {
        return;
    }

    BaseNode_Stop(&safety->base);
    printf("[%s] Nó de segurança parado\n", safety->base.node_id);
}

void SafetyNode_TriggerEmergency(SafetyNode_t *safety) {
    if (safety == NULL || safety->emergency_stop_active) {
        return;
    }

    safety->emergency_stop_active = true;
    safety->emergency_timestamp_us = BaseNode_GetTimestampUs();
    safety->emergency_activations++;
    g_emergency_stop = true;

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║                   ⚠️  EMERGÊNCIA ACIONADA ⚠️               ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  [%s] BOTÃO VERMELHO DE SEGURANÇA PRESSIONADO          ║\n", safety->base.node_id);
    printf("║  TODOS OS SISTEMAS SERÃO PARADOS IMEDIATAMENTE            ║\n");
    printf("║  Timestamp: %llu us                                     ║\n",
           (unsigned long long)safety->emergency_timestamp_us);
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    // Notifica via callback
    if (safety->on_emergency_callback != NULL) {
        safety->on_emergency_callback();
    }

    // Envia mensagem de parada para todos os nós
    Message_t stop_msg;
    stop_msg.type = MSG_HEARTBEAT; // Reutiliza tipo existente
    stop_msg.data[0] = 0xFF; // Código especial de emergência
    stop_msg.data_len = 1;

    BaseNode_SendMessage(&safety->base, &stop_msg);
    safety->total_stops_issued++;
}

bool SafetyNode_ResetEmergency(SafetyNode_t *safety) {
    if (safety == NULL) {
        return false;
    }

    if (safety->emergency_stop_active) {
        printf("\n[%s] Resetando estado de emergência...\n", safety->base.node_id);
        printf("[%s] Verifique se é seguro reiniciar o sistema!\n",
               safety->base.node_id);

        safety->emergency_stop_active = false;
        safety->button_pressed = false;
        g_emergency_stop = false;

        printf("[%s] ✓ Sistema liberado para operação\n\n", safety->base.node_id);
        return true;
    }

    return false;
}

bool SafetyNode_IsEmergencyActive(SafetyNode_t *safety) {
    if (safety == NULL) {
        return false;
    }
    return safety->emergency_stop_active;
}

void SafetyNode_RegisterEmergencyCallback(SafetyNode_t *safety, void (*callback)(void)) {
    if (safety != NULL) {
        safety->on_emergency_callback = callback;
    }
}

void SafetyNode_PrintStats(SafetyNode_t *safety) {
    if (safety == NULL) {
        return;
    }

    printf("\n[%s] ===== ESTATÍSTICAS DE SEGURANÇA =====\n",
           safety->base.node_id);
    printf("[%s] Estado: %s\n",
           safety->base.node_id,
           safety->emergency_stop_active ? "⚠️  EMERGÊNCIA ATIVA" : "✓ Operacional");
    printf("[%s] Total de acionamentos: %u\n",
           safety->base.node_id, safety->emergency_activations);
    printf("[%s] Comandos de parada emitidos: %u\n",
           safety->base.node_id, safety->total_stops_issued);

    if (safety->emergency_timestamp_us > 0) {
        printf("[%s] Último acionamento: %llu us\n",
               safety->base.node_id,
               (unsigned long long)safety->emergency_timestamp_us);
    }

    printf("[%s] ========================================\n\n",
           safety->base.node_id);
}

// ============================================================================
// FUNÇÃO AUXILIAR GLOBAL
// ============================================================================

/**
 * @brief Verifica se há parada de emergência ativa (para uso por outros nós)
 *
 * @return true se emergência ativa, false caso contrário
 */
bool SafetyNode_GlobalEmergencyCheck(void) {
    return g_emergency_stop;
}
