#include "conveyor_node.h"
#include "safety_node.h"
#include <string.h>
#include <stdio.h>

// Declaração externa da função de verificação de emergência
extern bool SafetyNode_GlobalEmergencyCheck(void);

// ============================================================================
// TAREFA DE CONTROLE DA ESTEIRA
// ============================================================================

static void ConveyorNode_ControlTask(void *pvParameters) {
    ConveyorNode_t *conveyor = (ConveyorNode_t *)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Atualiza a cada 100ms

    printf("[%s] Tarefa de controle da esteira iniciada\n", conveyor->base.node_id);

    while (conveyor->base.running) {
        // Verifica se há parada de emergência
        if (SafetyNode_GlobalEmergencyCheck() && conveyor->state != CONVEYOR_EMERGENCY_STOP) {
            printf("[%s] ⚠️  EMERGÊNCIA DETECTADA - PARANDO ESTEIRA IMEDIATAMENTE ⚠️\n",
                   conveyor->base.node_id);
            ConveyorNode_Halt(conveyor, true);
        }

        // Atualiza tempo de execução se estiver rodando
        if (conveyor->state == CONVEYOR_RUNNING) {
            uint64_t current_time = BaseNode_GetTimestampUs();
            conveyor->total_runtime_us += (current_time - conveyor->start_time_us);
            conveyor->start_time_us = current_time;
        }

        vTaskDelay(xFrequency);
    }

    vTaskDelete(NULL);
}

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

static void ConveyorNode_MotorControl(ConveyorNode_t *conveyor, bool enable) {
    // Em hardware real, controla PWM do motor
    // Exemplo para ESP32:
    // if (enable) {
    //     ledc_set_duty(LEDC_HIGH_SPEED_MODE, conveyor->motor_pwm_channel, duty);
    //     ledc_update_duty(LEDC_HIGH_SPEED_MODE, conveyor->motor_pwm_channel);
    // } else {
    //     ledc_set_duty(LEDC_HIGH_SPEED_MODE, conveyor->motor_pwm_channel, 0);
    //     ledc_update_duty(LEDC_HIGH_SPEED_MODE, conveyor->motor_pwm_channel);
    // }

    // Simulação
    if (enable) {
        printf("[%s] Motor da esteira LIGADO (velocidade: %u mm/s)\n",
               conveyor->base.node_id, conveyor->speed_mm_s);
    } else {
        printf("[%s] Motor da esteira DESLIGADO\n", conveyor->base.node_id);
    }
}

// ============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// ============================================================================

bool ConveyorNode_Init(ConveyorNode_t *conveyor, const char *node_id,
                      uint8_t motor_gpio, uint32_t speed_mm_s) {
    if (conveyor == NULL || node_id == NULL) {
        return false;
    }

    // Inicializa nó base
    if (!BaseNode_Init(&conveyor->base, node_id, NODE_TYPE_MONITOR)) {
        return false;
    }

    // Configurações iniciais
    conveyor->state = CONVEYOR_STOPPED;
    conveyor->speed_mm_s = speed_mm_s;
    conveyor->start_time_us = 0;
    conveyor->total_runtime_us = 0;

    // Estatísticas
    conveyor->objects_transported = 0;
    conveyor->total_stops = 0;
    conveyor->emergency_stops = 0;

    // GPIO
    conveyor->motor_gpio = motor_gpio;
    conveyor->motor_pwm_channel = 0;

    // Atualiza velocidade no nó base
    conveyor->base.belt_speed_mm_s = speed_mm_s;

    printf("[%s] Esteira inicializada (GPIO: %u, Velocidade: %u mm/s)\n",
           conveyor->base.node_id, motor_gpio, speed_mm_s);

    return true;
}

bool ConveyorNode_Start(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return false;
    }

    // Inicia nó base
    if (!BaseNode_Start(&conveyor->base)) {
        return false;
    }

    // Cria tarefa de controle com prioridade BAIXA (2)
    BaseTaskCreate(ConveyorNode_ControlTask, "ConveyorCtrl", 2048, conveyor, 2,
                   &conveyor->control_task_handle);

    if (conveyor->control_task_handle == NULL) {
        printf("[%s] ERRO: Falha ao criar tarefa de controle\n",
               conveyor->base.node_id);
        BaseNode_Stop(&conveyor->base);
        return false;
    }

    printf("[%s] Nó da esteira iniciado com prioridade BAIXA (2)\n",
           conveyor->base.node_id);

    return true;
}

void ConveyorNode_Stop(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return;
    }

    // Para a esteira primeiro
    ConveyorNode_Halt(conveyor, false);

    BaseNode_Stop(&conveyor->base);
    printf("[%s] Nó da esteira parado\n", conveyor->base.node_id);
}

bool ConveyorNode_Run(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return false;
    }

    // Verifica se não está em emergência
    if (SafetyNode_GlobalEmergencyCheck()) {
        printf("[%s] ERRO: Não é possível iniciar - sistema em EMERGÊNCIA\n",
               conveyor->base.node_id);
        return false;
    }

    if (conveyor->state == CONVEYOR_RUNNING) {
        return true; // Já está rodando
    }

    conveyor->state = CONVEYOR_RUNNING;
    conveyor->start_time_us = BaseNode_GetTimestampUs();

    ConveyorNode_MotorControl(conveyor, true);

    printf("[%s] >>> ESTEIRA INICIADA <<<\n", conveyor->base.node_id);
    printf("[%s] Velocidade: %u mm/s | Tempo de percurso (200mm): %u ms\n",
           conveyor->base.node_id,
           conveyor->speed_mm_s,
           ConveyorNode_CalculateTravelTime(conveyor, 200) / 1000);

    return true;
}

bool ConveyorNode_Halt(ConveyorNode_t *conveyor, bool emergency) {
    if (conveyor == NULL) {
        return false;
    }

    if (conveyor->state == CONVEYOR_STOPPED && !emergency) {
        return true; // Já está parada
    }

    ConveyorState_t previous_state = conveyor->state;
    conveyor->state = emergency ? CONVEYOR_EMERGENCY_STOP : CONVEYOR_STOPPED;

    ConveyorNode_MotorControl(conveyor, false);

    conveyor->total_stops++;
    if (emergency) {
        conveyor->emergency_stops++;
        printf("[%s] ⚠️⚠️⚠️  ESTEIRA PARADA POR EMERGÊNCIA  ⚠️⚠️⚠️\n",
               conveyor->base.node_id);
    } else {
        printf("[%s] Esteira parada\n", conveyor->base.node_id);
    }

    return true;
}

bool ConveyorNode_SetSpeed(ConveyorNode_t *conveyor, uint32_t speed_mm_s) {
    if (conveyor == NULL || speed_mm_s == 0) {
        return false;
    }

    conveyor->speed_mm_s = speed_mm_s;
    conveyor->base.belt_speed_mm_s = speed_mm_s;

    printf("[%s] Velocidade ajustada para %u mm/s\n",
           conveyor->base.node_id, speed_mm_s);

    // Se estiver rodando, atualiza motor
    if (conveyor->state == CONVEYOR_RUNNING) {
        ConveyorNode_MotorControl(conveyor, true);
    }

    return true;
}

uint32_t ConveyorNode_GetSpeed(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return 0;
    }
    return conveyor->speed_mm_s;
}

ConveyorState_t ConveyorNode_GetState(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return CONVEYOR_STOPPED;
    }
    return conveyor->state;
}

void ConveyorNode_NotifyObjectTransported(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return;
    }
    conveyor->objects_transported++;
}

uint32_t ConveyorNode_CalculateTravelTime(ConveyorNode_t *conveyor, uint32_t distance_mm) {
    if (conveyor == NULL || conveyor->speed_mm_s == 0) {
        return 0;
    }

    // Tempo = Distância / Velocidade (em microssegundos)
    float time_seconds = (float)distance_mm / (float)conveyor->speed_mm_s;
    return (uint32_t)(time_seconds * 1000000.0f);
}

void ConveyorNode_PrintStats(ConveyorNode_t *conveyor) {
    if (conveyor == NULL) {
        return;
    }

    const char *state_str;
    switch (conveyor->state) {
        case CONVEYOR_RUNNING:
            state_str = "Em movimento";
            break;
        case CONVEYOR_EMERGENCY_STOP:
            state_str = "⚠️  PARADA DE EMERGÊNCIA";
            break;
        case CONVEYOR_STOPPED:
        default:
            state_str = "Parada";
            break;
    }

    float runtime_seconds = conveyor->total_runtime_us / 1000000.0f;

    printf("\n[%s] ===== ESTATÍSTICAS DA ESTEIRA =====\n",
           conveyor->base.node_id);
    printf("[%s] Estado: %s\n", conveyor->base.node_id, state_str);
    printf("[%s] Velocidade: %u mm/s\n", conveyor->base.node_id, conveyor->speed_mm_s);
    printf("[%s] Objetos transportados: %u\n",
           conveyor->base.node_id, conveyor->objects_transported);
    printf("[%s] Total de paradas: %u\n",
           conveyor->base.node_id, conveyor->total_stops);
    printf("[%s] Paradas de emergência: %u\n",
           conveyor->base.node_id, conveyor->emergency_stops);
    printf("[%s] Tempo total de operação: %.1f s\n",
           conveyor->base.node_id, runtime_seconds);
    printf("[%s] Tempo de percurso (200mm): %u ms\n",
           conveyor->base.node_id,
           ConveyorNode_CalculateTravelTime(conveyor, 200) / 1000);
    printf("[%s] ========================================\n\n",
           conveyor->base.node_id);
}
