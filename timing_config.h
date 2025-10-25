/**
 * @file timing_config.h
 * @brief Configurações de timing realistas para o sistema de esteira industrial
 *
 * Este arquivo define todos os parâmetros temporais críticos do sistema,
 * baseados em especificações típicas de esteiras industriais e sistemas
 * de visão computacional embarcados.
 */

#ifndef TIMING_CONFIG_H
#define TIMING_CONFIG_H

#include <stdint.h>

// ============================================================================
// PARÂMETROS FÍSICOS DA ESTEIRA
// ============================================================================

/**
 * Velocidade típica de esteira industrial para inspeção de peças pequenas
 * Faixa típica: 50-200 mm/s
 * Valor escolhido: 100 mm/s (0.1 m/s) - velocidade moderada para boa detecção
 */
#define BELT_SPEED_MM_S                 100

/**
 * Distância entre câmera e pistão
 * Baseado em layout típico de esteira industrial
 * Faixa típica: 150-300 mm
 * Valor escolhido: 200 mm (permite tempo adequado para processamento)
 */
#define CAMERA_TO_PISTON_DISTANCE_MM    200

/**
 * Tempo de viagem do objeto da câmera até o pistão
 * Cálculo: distância / velocidade
 * = 200 mm / 100 mm/s = 2.0 segundos = 2,000,000 us
 */
#define BELT_TRAVEL_TIME_US             2000000UL  // 2 segundos

/**
 * Margem de segurança para compensar jitter e variações
 * 10% do tempo de viagem
 */
#define SAFETY_MARGIN_US                200000UL   // 200 ms

// ============================================================================
// TEMPOS DE PROCESSAMENTO - CÂMERA
// ============================================================================

/**
 * Tempo de captura de frame da câmera
 * Baseado em ESP32-CAM @ 10 FPS (típico para processamento em tempo real)
 * = 1000 ms / 10 FPS = 100 ms
 */
#define CAMERA_FRAME_CAPTURE_TIME_US    100000UL   // 100 ms

/**
 * Tempo de processamento de imagem (detecção de cor)
 * Inclui: conversão de espaço de cor, threshold, análise de pixels
 * Baseado em processamento otimizado no ESP32 (240 MHz dual-core)
 * Estimativa para frame 320x240: 50-150 ms
 */
#define IMAGE_PROCESSING_TIME_US        80000UL    // 80 ms

/**
 * Tempo total de detecção (captura + processamento)
 */
#define TOTAL_DETECTION_TIME_US         (CAMERA_FRAME_CAPTURE_TIME_US + IMAGE_PROCESSING_TIME_US)  // 180 ms

/**
 * Intervalo entre frames (para evitar sobrecarga)
 * Taxa de aquisição: ~5 FPS para análise estável
 */
#define CAMERA_FRAME_INTERVAL_US        200000UL   // 200 ms (5 FPS)

/**
 * Cooldown entre detecções (para evitar duplicatas do mesmo objeto)
 * Deve ser > tempo que objeto leva para sair do campo de visão
 * Baseado em objeto de 50mm @ 100mm/s = 500ms
 */
#define DETECTION_COOLDOWN_US           600000UL   // 600 ms

// ============================================================================
// TEMPOS DE COMUNICAÇÃO
// ============================================================================

/**
 * Latência de comunicação inter-nós via queue FreeRTOS
 * Comunicação local: praticamente instantânea (< 1ms)
 * Incluindo overhead de context switch
 */
#define QUEUE_SEND_LATENCY_US           1000UL     // 1 ms

/**
 * Latência máxima de comunicação em rede (se usar WiFi/UDP)
 * Baseado em rede local com ESP32
 * LAN típica: 1-10 ms
 */
#define NETWORK_LATENCY_MAX_US          10000UL    // 10 ms

/**
 * Timeout para operações de queue
 */
#define QUEUE_TIMEOUT_US                50000UL    // 50 ms

// ============================================================================
// TEMPOS DE ATUAÇÃO - PISTÃO
// ============================================================================

/**
 * Tempo de acionamento físico do pistão pneumático
 * Inclui:
 * - Tempo de resposta da válvula solenoide: ~5-15 ms
 * - Tempo de extensão do pistão: ~50-200 ms (depende do curso)
 * - Tempo total típico: 60-150 ms
 */
#define PISTON_ACTIVATION_TIME_US       100000UL   // 100 ms

/**
 * Tempo de retorno do pistão à posição inicial
 */
#define PISTON_RETRACTION_TIME_US       100000UL   // 100 ms

/**
 * Tempo mínimo entre ativações consecutivas do mesmo pistão
 * Para evitar sobrecarga mecânica
 */
#define PISTON_COOLDOWN_US              300000UL   // 300 ms

/**
 * Tempo de estabilização após ativação
 */
#define PISTON_SETTLE_TIME_US           50000UL    // 50 ms

// ============================================================================
// DEADLINES DE TEMPO REAL
// ============================================================================

/**
 * Deadline end-to-end: detecção -> ativação do pistão
 * Deve ser menor que o tempo de viagem na esteira
 *
 * Tempo disponível: BELT_TRAVEL_TIME_US = 2,000 ms
 * Tempo usado:
 *   - Detecção: 180 ms
 *   - Comunicação: 1 ms (local) ou 10 ms (rede)
 *   - Processamento pistão: 10 ms
 *   - Ativação física: 100 ms
 *   - Margem de segurança: 200 ms
 * Total: ~500 ms
 *
 * Deadline conservador: 1,500 ms (75% do tempo disponível)
 */
#define END_TO_END_DEADLINE_US          1500000UL  // 1.5 segundos

/**
 * Deadline para processamento de imagem
 * Deve completar antes do próximo frame
 */
#define IMAGE_PROCESSING_DEADLINE_US    150000UL   // 150 ms

/**
 * Deadline para ativação do pistão (após receber comando)
 * Inclui tempo de resposta do sistema
 */
#define PISTON_RESPONSE_DEADLINE_US     50000UL    // 50 ms

/**
 * Deadline crítico: tempo máximo antes de perder o objeto
 * Se ultrapassar, o objeto já passou do pistão
 */
#define CRITICAL_DEADLINE_US            BELT_TRAVEL_TIME_US

// ============================================================================
// PERÍODOS DE TAREFAS FREERTOS
// ============================================================================

/**
 * Período da tarefa de captura de câmera (alta prioridade)
 * Deve capturar frames continuamente
 */
#define CAMERA_TASK_PERIOD_MS           200        // 200 ms (5 FPS)

/**
 * Período da tarefa de processamento de imagem
 * Triggered por evento de frame capturado
 */
#define PROCESSING_TASK_PERIOD_MS       10         // 10 ms (polling rápido)

/**
 * Período da tarefa de controle do pistão
 * Verifica fila de comandos frequentemente
 */
#define PISTON_TASK_PERIOD_MS           5          // 5 ms (alta responsividade)

/**
 * Período da tarefa de heartbeat
 */
#define HEARTBEAT_TASK_PERIOD_MS        1000       // 1 segundo

/**
 * Período da tarefa de monitoramento/métricas
 */
#define MONITOR_TASK_PERIOD_MS          100        // 100 ms

/**
 * Período da tarefa de estatísticas
 */
#define STATS_TASK_PERIOD_MS            10000      // 10 segundos

// ============================================================================
// PRIORIDADES DE TAREFAS FREERTOS
// ============================================================================

/**
 * Sistema de prioridades (0 = menor, 5 = maior)
 *
 * Princípio: Tarefas críticas de tempo real têm maior prioridade
 */

// Prioridade máxima: Thread Master de Segurança (acima do scheduler!)
// Esta thread fica embaixo apenas do escalonador do FreeRTOS
// Quando botão de emergência é acionado, para TUDO imediatamente
#define PRIORITY_EMERGENCY              10         // MÁXIMA - Thread Master

// Prioridade alta: Tarefas de tempo real críticas
#define PRIORITY_PISTON_CONTROL         4          // Deve responder imediatamente
#define PRIORITY_IMAGE_PROCESSING       4          // Detecção não pode atrasar

// Prioridade média-alta: Tarefas importantes mas com alguma margem
#define PRIORITY_CAMERA_CAPTURE         3          // Captura pode ter pequeno jitter
#define PRIORITY_COMMUNICATION_RX       3          // Recepção de mensagens

// Prioridade média: Tarefas regulares
#define PRIORITY_COMMUNICATION_TX       2          // Envio pode aguardar
#define PRIORITY_HEARTBEAT              2          // Não é crítico

// Prioridade baixa: Tarefas de background
#define PRIORITY_MONITOR                1          // Monitoramento
#define PRIORITY_STATS                  1          // Estatísticas

// Prioridade mínima: Idle
#define PRIORITY_IDLE                   0

// ============================================================================
// TAMANHOS DE PILHA (STACK) DAS TAREFAS
// ============================================================================

/**
 * Tamanhos em words (4 bytes no ESP32)
 * Baseado em análise de uso típico + margem de segurança
 */

#define STACK_SIZE_CAMERA_CAPTURE       4096       // Buffers de imagem grandes
#define STACK_SIZE_IMAGE_PROCESSING     8192       // Processamento intensivo
#define STACK_SIZE_PISTON_CONTROL       2048       // Controle simples
#define STACK_SIZE_COMMUNICATION        2048       // TX/RX de mensagens
#define STACK_SIZE_HEARTBEAT            1024       // Tarefa leve
#define STACK_SIZE_MONITOR              2048       // Coleta de métricas
#define STACK_SIZE_STATS                4096       // Impressão de strings

// ============================================================================
// TAMANHOS DE FILAS (QUEUES)
// ============================================================================

/**
 * Número de elementos que cada fila pode armazenar
 */

#define QUEUE_SIZE_RX                   10         // Mensagens recebidas
#define QUEUE_SIZE_TX                   10         // Mensagens a enviar
#define QUEUE_SIZE_DETECTION_EVENTS     5          // Eventos de detecção
#define QUEUE_SIZE_PISTON_COMMANDS      5          // Comandos de pistão

// ============================================================================
// LIMIARES E CONFIGURAÇÕES
// ============================================================================

/**
 * Número mínimo de pixels para confirmar detecção de cor
 * Baseado em frame 320x240 = 76,800 pixels totais
 * Objeto ocupando ~5-10% da imagem = 3,840-7,680 pixels
 */
#define MIN_COLOR_PIXELS                3000       // ~4% da imagem

/**
 * Qualidade mínima de detecção (0.0 a 1.0)
 * Baseado em confiança do algoritmo de visão
 */
#define MIN_DETECTION_QUALITY           0.70f      // 70% de confiança

/**
 * Jitter máximo aceitável (variação de latência)
 */
#define MAX_ACCEPTABLE_JITTER_US        50000UL    // 50 ms

/**
 * QoS mínimo aceitável (0.0 a 1.0)
 */
#define MIN_ACCEPTABLE_QOS              0.90f      // 90%

// ============================================================================
// CÁLCULOS DERIVADOS (NÃO MODIFICAR)
// ============================================================================

/**
 * Tempo total do pipeline crítico (detecção até ativação)
 * Usado para validar que deadline é atingível
 */
#define PIPELINE_TOTAL_TIME_US          (TOTAL_DETECTION_TIME_US + \
                                         QUEUE_SEND_LATENCY_US + \
                                         PISTON_ACTIVATION_TIME_US)

/**
 * Margem de tempo disponível após pipeline
 */
#define TIME_MARGIN_US                  (END_TO_END_DEADLINE_US - PIPELINE_TOTAL_TIME_US)

/**
 * Validação em compile-time: pipeline deve caber no deadline
 */
#if PIPELINE_TOTAL_TIME_US > END_TO_END_DEADLINE_US
    #error "Pipeline time exceeds deadline! Adjust timing parameters."
#endif

/**
 * Validação: deadline não pode exceder tempo de viagem
 */
#if END_TO_END_DEADLINE_US >= BELT_TRAVEL_TIME_US
    #warning "Deadline is very close to travel time. Consider reducing for safety margin."
#endif

// ============================================================================
// FUNÇÕES AUXILIARES
// ============================================================================

/**
 * @brief Calcula tempo de viagem baseado na distância e velocidade
 *
 * @param distance_mm Distância em milímetros
 * @return Tempo em microssegundos
 */
static inline uint32_t CalculateTravelTime(uint32_t distance_mm) {
    return (distance_mm * 1000000UL) / BELT_SPEED_MM_S;
}

/**
 * @brief Verifica se latência está dentro do deadline
 *
 * @param latency_us Latência medida em microssegundos
 * @return true se dentro do deadline, false caso contrário
 */
static inline bool IsWithinDeadline(uint32_t latency_us) {
    return latency_us <= END_TO_END_DEADLINE_US;
}

/**
 * @brief Calcula QoS baseado na latência
 *
 * @param latency_us Latência medida em microssegundos
 * @return Pontuação de QoS (0.0 a 1.0)
 */
static inline float CalculateQoS(uint32_t latency_us) {
    if (latency_us <= END_TO_END_DEADLINE_US) {
        // Dentro do deadline: QoS baseado em quão perto do deadline está
        // Latência menor = QoS maior
        float ratio = (float)latency_us / (float)END_TO_END_DEADLINE_US;
        return 1.0f - (ratio * 0.2f);  // QoS entre 0.8 e 1.0
    } else {
        // Perdeu deadline: QoS baseado em quão tarde está
        float overtime = (float)(latency_us - END_TO_END_DEADLINE_US);
        float penalty = overtime / (float)END_TO_END_DEADLINE_US;
        float qos = 0.8f - penalty;  // Penalidade progressiva
        return (qos < 0.0f) ? 0.0f : qos;
    }
}

// ============================================================================
// INFORMAÇÕES DO SISTEMA
// ============================================================================

/**
 * Análise de timing do sistema:
 *
 * CENÁRIO NORMAL (sem atrasos):
 * 1. Objeto entra no campo da câmera: T0
 * 2. Frame capturado: T0 + 100 ms
 * 3. Cor detectada: T0 + 180 ms
 * 4. Comando enviado ao pistão: T0 + 181 ms
 * 5. Pistão acionado: T0 + 281 ms
 * 6. Objeto alcança pistão: T0 + 2000 ms
 *
 * Margem de segurança: 2000 - 281 = 1719 ms (85.9% do tempo disponível)
 *
 * CENÁRIO WORST-CASE (com jitter máximo):
 * - Detecção atrasada: +50 ms
 * - Comunicação atrasada: +10 ms (rede)
 * - Ativação atrasada: +20 ms
 * Total: 180 + 50 + 10 + 100 + 20 = 360 ms
 * Margem: 2000 - 360 = 1640 ms (82% do tempo disponível)
 *
 * CONCLUSÃO: Sistema tem margem robusta para variações.
 */

#endif // TIMING_CONFIG_H
