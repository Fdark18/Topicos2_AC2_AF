/**
 * @file main.c
 * @brief Sistema de Esteira Industrial com FreeRTOS
 *
 * Implementação de sistema distribuído em tempo real para classificação
 * automática de objetos por cor usando FreeRTOS.
 *
 * Componentes:
 * - 1 Nó Câmera: Detecta cores (azul/verde)
 * - 2 Nós Pistão: Empurram objetos (Pistão A: verde->direita, Pistão B: azul->esquerda)
 *
 * Este exemplo demonstra:
 * - Multitarefa com FreeRTOS
 * - Comunicação entre nós via queues
 * - Timing de tempo real com deadlines
 * - Métricas de QoS e latência
 *
 * ORDEM DE INICIALIZAÇÃO:
 * 1. Inicialização de hardware (GPIO, periféricos)
 * 2. Criação de recursos compartilhados (queues, semáforos, mutexes)
 * 3. Inicialização dos nós (camera, pistons)
 * 4. Criação de tarefas por ordem de prioridade (alta -> baixa)
 * 5. Início do scheduler (ou automático no ESP-IDF)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "camera_node.h"
#include "piston_node.h"
#include "conveyor_node.h"
#include "safety_node.h"
#include "timing_config.h"
#include <stdio.h>

// ============================================================================
// INSTÂNCIAS DOS NÓS
// ============================================================================

static SafetyNode_t safety;      // Nó de emergência (prioridade MÁXIMA)
static ConveyorNode_t conveyor;  // Nó da esteira
static CameraNode_t camera;      // Nó da câmera
static PistonNode_t piston_a;    // Pistão A (verde)
static PistonNode_t piston_b;    // Pistão B (azul)

// ============================================================================
// TAREFA DE SIMULAÇÃO DE DETECÇÕES
// ============================================================================

/**
 * @brief Tarefa que simula detecções de cores para teste
 *
 * Em sistema real, esta tarefa seria substituída por captura de câmera real
 *
 * Prioridade: BAIXA (não é crítica para tempo real)
 * Período: 5 segundos entre detecções
 */
static void SimulationTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); // 5 segundos

    printf("\n[SIMULACAO] Tarefa iniciada - Simulando detecções a cada 5s\n");
    printf("[SIMULACAO] Sequência: VERDE -> AZUL -> VERDE -> AZUL...\n\n");

    vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2s antes de começar

    uint8_t sequence = 0;

    while (1) {
        if (sequence % 2 == 0) {
            // Simula detecção de VERDE
            printf("[SIMULACAO] ===== Simulando detecção de VERDE =====\n");
            CameraNode_SimulateDetection(&camera, COLOR_GREEN, 500, 0.85f);
        } else {
            // Simula detecção de AZUL
            printf("[SIMULACAO] ===== Simulando detecção de AZUL =====\n");
            CameraNode_SimulateDetection(&camera, COLOR_BLUE, 450, 0.90f);
        }

        sequence++;
        vTaskDelay(xFrequency);
    }
}

// ============================================================================
// TAREFA DE ESTATÍSTICAS
// ============================================================================

/**
 * @brief Tarefa que imprime estatísticas periodicamente
 *
 * Prioridade: BAIXA (monitoramento, não crítica)
 * Período: 15 segundos
 */
static void StatsTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(STATS_TASK_PERIOD_MS); // Configurado em timing_config.h

    vTaskDelay(pdMS_TO_TICKS(10000)); // Aguarda 10s antes da primeira impressão

    while (1) {
        printf("\n");
        printf("╔════════════════════════════════════════════════════════════╗\n");
        printf("║         RELATORIO DE ESTATISTICAS DO SISTEMA              ║\n");
        printf("╚════════════════════════════════════════════════════════════╝\n");

        SafetyNode_PrintStats(&safety);
        ConveyorNode_PrintStats(&conveyor);
        CameraNode_PrintStats(&camera);
        PistonNode_PrintStats(&piston_a);
        PistonNode_PrintStats(&piston_b);

        vTaskDelay(xFrequency);
    }
}

// ============================================================================
// FUNÇÃO DE INICIALIZAÇÃO DO SISTEMA
// ============================================================================

/**
 * @brief Inicializa hardware e periféricos
 *
 * Esta função deve ser chamada ANTES de qualquer outra inicialização
 *
 * @return true se sucesso, false se falha
 */
static bool InitializeHardware(void) {
    printf("[INIT] Inicializando hardware...\n");

    // Aqui entrariam inicializações de hardware específicas:
    // - GPIO pins
    // - PWM para motores
    // - I2C/SPI para sensores
    // - Camera interface
    // - Timer hardware

    printf("[INIT] Hardware inicializado\n");
    return true;
}

/**
 * @brief Inicializa todos os nós do sistema
 *
 * ORDEM CRÍTICA:
 * 1. Hardware já deve estar inicializado
 * 2. Inicializar nós de baixo para cima (pistões -> câmera)
 * 3. Recursos compartilhados são criados automaticamente pelos nós
 *
 * @return true se sucesso, false se falha
 */
static bool InitializeSystem(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║   SISTEMA DE ESTEIRA INDUSTRIAL - MVP FreeRTOS            ║\n");
    printf("║   Sistema Distribuído de Tempo Real com Segurança         ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    // Inicializa hardware primeiro
    if (!InitializeHardware()) {
        printf("[MAIN] ERRO: Falha na inicialização do hardware\n");
        return false;
    }

    // Portas dos pistões (para comunicação simulada)
    uint16_t piston_ports[] = {5002, 5003};

    // ORDEM CRÍTICA DE INICIALIZAÇÃO:
    // 1. Safety Node (PRIMEIRO - controla tudo)
    // 2. Conveyor Node (esteira)
    // 3. Pistões (consumidores)
    // 4. Câmera (produtor)

    // 1. Inicializa Nó de Segurança PRIMEIRO (máxima prioridade)
    printf("[MAIN] Inicializando Nó de Segurança (PRIORIDADE MÁXIMA)...\n");
    if (!SafetyNode_Init(&safety, "safety_master", 0)) {
        printf("[MAIN] ERRO: Falha ao inicializar nó de segurança\n");
        return false;
    }

    // 2. Inicializa Nó da Esteira
    printf("[MAIN] Inicializando Nó da Esteira...\n");
    if (!ConveyorNode_Init(&conveyor, "conveyor_belt", 0, BELT_SPEED_MM_S)) {
        printf("[MAIN] ERRO: Falha ao inicializar esteira\n");
        return false;
    }

    // 3. Inicializa Pistão A (VERDE -> DIREITA)
    printf("[MAIN] Inicializando Pistão A (VERDE -> DIREITA)...\n");
    if (!PistonNode_CreatePistonA(&piston_a)) {
        printf("[MAIN] ERRO: Falha ao inicializar Pistão A\n");
        return false;
    }

    // 4. Inicializa Pistão B (AZUL -> ESQUERDA)
    printf("[MAIN] Inicializando Pistão B (AZUL -> ESQUERDA)...\n");
    if (!PistonNode_CreatePistonB(&piston_b)) {
        printf("[MAIN] ERRO: Falha ao inicializar Pistão B\n");
        return false;
    }

    // 5. Inicializa nó da câmera por último (produtor)
    printf("[MAIN] Inicializando nó da câmera...\n");
    if (!CameraNode_Init(&camera, "camera_1", piston_ports, 2)) {
        printf("[MAIN] ERRO: Falha ao inicializar câmera\n");
        return false;
    }

    printf("[MAIN] Todos os nós inicializados com sucesso\n\n");
    return true;
}

/**
 * @brief Inicia todos os nós do sistema
 *
 * ORDEM CRÍTICA:
 * 1. Inicializar pistões primeiro (consumidores devem estar prontos)
 * 2. Inicializar câmera por último (produtor começa a gerar eventos)
 *
 * Isso evita race conditions onde a câmera detecta algo antes
 * dos pistões estarem prontos para processar.
 *
 * @return true se sucesso, false se falha
 */
static bool StartSystem(void) {
    printf("[MAIN] Iniciando sistema...\n\n");

    // ORDEM CRÍTICA: Maior prioridade primeiro
    // 1. Safety Node (prioridade MÁXIMA 10)
    // 2. Conveyor, Pistões, Câmera

    // 1. Inicia Nó de Segurança PRIMEIRO (prioridade 10 - MÁXIMA)
    printf("[MAIN] Iniciando Nó de Segurança (Prioridade: 10 - MÁXIMA)...\n");
    if (!SafetyNode_Start(&safety)) {
        printf("[MAIN] ERRO: Falha ao iniciar nó de segurança\n");
        return false;
    }

    // 2. Inicia Esteira (prioridade 2)
    printf("[MAIN] Iniciando Esteira (Prioridade: 2)...\n");
    if (!ConveyorNode_Start(&conveyor)) {
        printf("[MAIN] ERRO: Falha ao iniciar esteira\n");
        return false;
    }

    // 3. Inicia Pistão A
    printf("[MAIN] Iniciando Pistão A (Prioridade: %d)...\n", PRIORITY_PISTON_CONTROL);
    if (!PistonNode_Start(&piston_a)) {
        printf("[MAIN] ERRO: Falha ao iniciar Pistão A\n");
        return false;
    }

    // 4. Inicia Pistão B
    printf("[MAIN] Iniciando Pistão B (Prioridade: %d)...\n", PRIORITY_PISTON_CONTROL);
    if (!PistonNode_Start(&piston_b)) {
        printf("[MAIN] ERRO: Falha ao iniciar Pistão B\n");
        return false;
    }

    // 5. Inicia câmera por último (produtor)
    printf("[MAIN] Iniciando Câmera (Prioridade: %d)...\n", PRIORITY_IMAGE_PROCESSING);
    if (!CameraNode_Start(&camera)) {
        printf("[MAIN] ERRO: Falha ao iniciar câmera\n");
        return false;
    }

    // 6. Liga a esteira (após tudo estar pronto)
    printf("[MAIN] Ligando esteira...\n");
    if (!ConveyorNode_Run(&conveyor)) {
        printf("[MAIN] AVISO: Falha ao ligar esteira\n");
    }

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║              SISTEMA INICIADO COM SUCESSO                  ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  COMPONENTES ATIVOS:                                       ║\n");
    printf("║  - Safety:   safety_master (EMERGÊNCIA)                   ║\n");
    printf("║  - Esteira:  conveyor_belt (RODANDO)                      ║\n");
    printf("║  - Camera:   camera_1 (Detectando AZUL e VERDE)           ║\n");
    printf("║  - Pistão A: piston_a (VERDE -> DIREITA)                  ║\n");
    printf("║  - Pistão B: piston_b (AZUL -> ESQUERDA)                  ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  PARAMETROS DE TEMPO REAL:                                ║\n");
    printf("║  - Deadline end-to-end: %d ms                        ║\n", END_TO_END_DEADLINE_US / 1000);
    printf("║  - Velocidade Esteira: %d mm/s                        ║\n", BELT_SPEED_MM_S);
    printf("║  - Distância Câmera-Pistão: %d mm                      ║\n", CAMERA_TO_PISTON_DISTANCE_MM);
    printf("║  - Tempo de viagem: %d ms                           ║\n", BELT_TRAVEL_TIME_US / 1000);
    printf("║  - Margem de segurança: %d ms                         ║\n", SAFETY_MARGIN_US / 1000);
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  PRIORIDADES DAS TAREFAS (FREERTOS):                      ║\n");
    printf("║  - Emergency Safety: 10 (MÁXIMA - acima do scheduler!)    ║\n");
    printf("║  - Pistão Control: %d (CRÍTICA)                            ║\n", PRIORITY_PISTON_CONTROL);
    printf("║  - Image Processing: %d (ALTA)                             ║\n", PRIORITY_IMAGE_PROCESSING);
    printf("║  - Camera Capture: %d (MÉDIA-ALTA)                         ║\n", PRIORITY_CAMERA_CAPTURE);
    printf("║  - Conveyor/Heartbeat: %d (MÉDIA)                          ║\n", PRIORITY_HEARTBEAT);
    printf("║  - Monitor/Stats: %d (BAIXA)                               ║\n", PRIORITY_MONITOR);
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    return true;
}

// ============================================================================
// FUNÇÃO MAIN
// ============================================================================

/**
 * @brief Função principal
 *
 * SEQUÊNCIA DE INICIALIZAÇÃO:
 * 1. Inicializar hardware (GPIO, periféricos)
 * 2. Inicializar nós do sistema (pistões -> câmera)
 * 3. Iniciar nós (cria tarefas FreeRTOS internas)
 * 4. Criar tarefas auxiliares (simulação, stats)
 * 5. Scheduler continua rodando (já iniciado no ESP-IDF)
 *
 * PRIORIDADES (0=baixa, 5=alta):
 * - Emergency: 5 (não usado neste exemplo)
 * - Piston Control: 4 (CRÍTICA - deve responder imediatamente)
 * - Image Processing: 4 (ALTA - detecção não pode atrasar)
 * - Camera Capture: 3 (MÉDIA-ALTA - pode ter pequeno jitter)
 * - Communication: 2-3 (MÉDIA)
 * - Heartbeat: 2 (MÉDIA)
 * - Simulation: 2 (MÉDIA - para este exemplo)
 * - Monitor/Stats: 1 (BAIXA - background)
 */
void app_main(void) {
    // Inicializa sistema (hardware + nós)
    if (!InitializeSystem()) {
        printf("[MAIN] ERRO FATAL: Falha na inicialização\n");
        return;
    }

    // Inicia sistema (cria tarefas FreeRTOS dos nós)
    // ORDEM: pistões primeiro, depois câmera
    if (!StartSystem()) {
        printf("[MAIN] ERRO FATAL: Falha ao iniciar sistema\n");
        return;
    }

    // Cria tarefas auxiliares
    // ORDEM: da maior para menor prioridade é boa prática

    printf("[MAIN] Criando tarefas auxiliares...\n");

    // Tarefa de simulação (prioridade 2 - média)
    // Em sistema real, esta seria a captura de câmera real
    TaskHandle_t sim_handle;
    BaseType_t result = xTaskCreate(
        SimulationTask,              // Função da tarefa
        "Simulation",                // Nome (para debug)
        2048,                        // Stack size (em words)
        NULL,                        // Parâmetros
        PRIORITY_COMMUNICATION_TX,   // Prioridade 2
        &sim_handle                  // Handle
    );

    if (result != pdPASS) {
        printf("[MAIN] ERRO: Falha ao criar tarefa de simulação\n");
        return;
    }

    // Tarefa de estatísticas (prioridade 1 - baixa)
    TaskHandle_t stats_handle;
    result = xTaskCreate(
        StatsTask,                   // Função da tarefa
        "Statistics",                // Nome
        STACK_SIZE_STATS,            // Stack configurado em timing_config.h
        NULL,                        // Parâmetros
        PRIORITY_STATS,              // Prioridade 1
        &stats_handle                // Handle
    );

    if (result != pdPASS) {
        printf("[MAIN] ERRO: Falha ao criar tarefa de estatísticas\n");
        return;
    }

    printf("[MAIN] Tarefas auxiliares criadas com sucesso\n\n");

    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║              SISTEMA EM OPERAÇÃO                           ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  Tarefas/Threads Ativas (ordem de prioridade):             ║\n");
    printf("║  1. Safety Monitor (P:10) - THREAD MASTER                  ║\n");
    printf("║  2. Piston A Control (P:%d) - Thread Atuador               ║\n", PRIORITY_PISTON_CONTROL);
    printf("║  3. Piston B Control (P:%d) - Thread Atuador               ║\n", PRIORITY_PISTON_CONTROL);
    printf("║  4. Camera Processing (P:%d) - Thread Sensor               ║\n", PRIORITY_IMAGE_PROCESSING);
    printf("║  5. Conveyor Control (P:%d) - Thread Esteira               ║\n", PRIORITY_HEARTBEAT);
    printf("║  6. Simulation (P:%d) - Thread Teste                       ║\n", PRIORITY_COMMUNICATION_TX);
    printf("║  7. Statistics (P:%d) - Thread Monitor                     ║\n", PRIORITY_STATS);
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  EMERGÊNCIA: Pressione 'E' para simular botão vermelho    ║\n");
    printf("║  O scheduler FreeRTOS está gerenciando as threads         ║\n");
    printf("║  Thread Master monitora emergência continuamente          ║\n");
    printf("║  Para parar: Ctrl+C ou reset do hardware                  ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n");

    // Em FreeRTOS/ESP-IDF, app_main não deve retornar
    // O scheduler já está rodando (iniciado automaticamente pelo ESP-IDF)
    // Esta função retorna mas as tarefas criadas continuam executando
}

// ============================================================================
// VERSÃO ALTERNATIVA PARA SISTEMAS SEM ESP-IDF
// ============================================================================

#ifdef USE_STANDARD_MAIN

/**
 * @brief Versão main() padrão para sistemas que não usam ESP-IDF
 *
 * Esta versão é para microcontroladores que requerem início manual
 * do scheduler FreeRTOS (ex: ARM Cortex-M, STM32, etc.)
 *
 * Diferenças do ESP-IDF:
 * - Requer vTaskStartScheduler() explícito
 * - main() nunca deve retornar após iniciar o scheduler
 * - Se retornar, indica erro fatal (memória insuficiente)
 */
int main(void) {
    // Inicializa sistema (hardware + nós)
    if (!InitializeSystem()) {
        printf("[MAIN] ERRO FATAL: Falha na inicialização\n");
        return -1;
    }

    // Inicia sistema (cria tarefas dos nós)
    if (!StartSystem()) {
        printf("[MAIN] ERRO FATAL: Falha ao iniciar sistema\n");
        return -1;
    }

    // Cria tarefas auxiliares
    printf("[MAIN] Criando tarefas auxiliares...\n");

    TaskHandle_t sim_handle, stats_handle;
    BaseType_t result;

    result = xTaskCreate(SimulationTask, "Simulation", 2048, NULL,
                        PRIORITY_COMMUNICATION_TX, &sim_handle);
    if (result != pdPASS) {
        printf("[MAIN] ERRO: Falha ao criar tarefa de simulação\n");
        return -1;
    }

    result = xTaskCreate(StatsTask, "Statistics", STACK_SIZE_STATS, NULL,
                        PRIORITY_STATS, &stats_handle);
    if (result != pdPASS) {
        printf("[MAIN] ERRO: Falha ao criar tarefa de estatísticas\n");
        return -1;
    }

    printf("[MAIN] Todas as tarefas criadas com sucesso\n");
    printf("[MAIN] Iniciando scheduler FreeRTOS...\n\n");

    // Inicia scheduler FreeRTOS
    // IMPORTANTE: Esta chamada só retorna se houver erro fatal
    // (geralmente falta de memória heap para criar a idle task)
    vTaskStartScheduler();

    // ========================================================================
    // NUNCA DEVE CHEGAR AQUI
    // Se chegou, houve erro crítico na inicialização do scheduler
    // ========================================================================
    printf("[MAIN] ERRO CRÍTICO: Scheduler retornou!\n");
    printf("[MAIN] Possíveis causas:\n");
    printf("  - Memória heap insuficiente (configTOTAL_HEAP_SIZE muito pequeno)\n");
    printf("  - Falha ao criar idle task\n");
    printf("  - Configuração incorreta do FreeRTOSConfig.h\n");

    return -1;
}

#endif // USE_STANDARD_MAIN
