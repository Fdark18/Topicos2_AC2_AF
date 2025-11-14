#include "freertos_sim.h"

#include "camera_node.h"
#include "conveyor_node.h"
#include "piston_node.h"
#include "safety_node.h"
#include "base_node.h"

#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
static void SetupConsoleUtf8(void) {
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
}
#else
static void SetupConsoleUtf8(void) {}
#endif

extern bool SafetyNode_GlobalEmergencyCheck(void);

typedef struct {
    BaseNode_t *source;
    BaseNode_t *targets[4];
    size_t target_count;
} RouterContext_t;

static SafetyNode_t safety;
static ConveyorNode_t conveyor;
static CameraNode_t camera;
static PistonNode_t piston_a;
static PistonNode_t piston_b;
static RouterContext_t camera_router_ctx;

static void SimulateColorPass(PistonNode_t *target_piston,
                              Color_t color,
                              uint32_t pixels,
                              float quality) {
    const char *color_name = BaseNode_ColorToString(color);
    printf("[SIMULACAO] >>> Objeto %s entrou na câmera - pistão %s (%s)\n",
           color_name, target_piston->base.node_id, target_piston->direction);

    uint64_t start_us = BaseNode_GetTimestampUs();
    CameraNode_SimulateDetection(&camera, color, pixels, quality);
    vTaskDelay(pdMS_TO_TICKS(1200));
    CameraNode_SimulateObjectExit(&camera);

    uint64_t total_us = BaseNode_GetTimestampUs() - start_us;
    printf("[SIMULACAO] <<< Objeto %s processado em %lu ms (deadline alvo %lu ms)\n",
           color_name,
           (unsigned long)(total_us / 1000ULL),
           (unsigned long)(target_piston->base.deadline_us / 1000ULL));
    vTaskDelay(pdMS_TO_TICKS(800)); // espaçamento para manter janela determinística
}

static void SimulateUnexpectedRedEvent(void) {
    printf("[SIMULACAO] >>> Objeto VERMELHO identificado - cor fora da tabela, operador acionará emergência.\n");
    safety.button_pressed = true;
    vTaskDelay(pdMS_TO_TICKS(2000)); // máquina parada para inspeção
    safety.button_pressed = false;
    printf("[SIMULACAO] >>> Operador iniciou reset do sistema após inspeção de segurança.\n");
    SafetyNode_ResetEmergency(&safety);
    ConveyorNode_Run(&conveyor);
    vTaskDelay(pdMS_TO_TICKS(500));
}
static void HostEmergencyCallback(void) {
    printf("[HOST] Callback recebido - garantindo parada da esteira.\n");
    ConveyorNode_Halt(&conveyor, true);
}

static void BlinkTask(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        printf("[BLINK] Heartbeat - tick=%lu ms\n", (unsigned long)xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void SimulationTask(void *pvParameters) {
    (void)pvParameters;

    printf("[SIMULACAO] Cenário determinístico: 2x azul -> pistão esquerda, 1x vermelho -> emergência, 2x verde -> pistão direita\n");
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Objetos 1 e 2: AZUL -> pistão esquerda
    SimulateColorPass(&piston_b, COLOR_BLUE, 500, 0.90f);
    SimulateColorPass(&piston_b, COLOR_BLUE, 470, 0.88f);

    // Objeto 3: VERMELHO -> emergência manual e reset
    SimulateUnexpectedRedEvent();

    // Objetos 4 e 5: VERDE -> pistão direita
    SimulateColorPass(&piston_a, COLOR_GREEN, 520, 0.93f);
    SimulateColorPass(&piston_a, COLOR_GREEN, 480, 0.91f);

    printf("[SIMULACAO] Sequência completa. Estatísticas refletirão 5 objetos e a pausa de emergência.\n");

    // Mantém tarefa viva para futuras execuções/observações
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void StatsTask(void *pvParameters) {
    (void)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(10000);

    vTaskDelay(pdMS_TO_TICKS(5000));
    while (1) {
        SafetyNode_PrintStats(&safety);
        ConveyorNode_PrintStats(&conveyor);
        CameraNode_PrintStats(&camera);
        PistonNode_PrintStats(&piston_a);
        PistonNode_PrintStats(&piston_b);
        float qos_dir = BaseNode_GetAverageQoS(&piston_a.base) * 100.0f;
        float qos_esq = BaseNode_GetAverageQoS(&piston_b.base) * 100.0f;
        printf("[STATS] Determinismo/QoS -> Pistao direita: %.1f%% | Pistao esquerda: %.1f%%\n",
               qos_dir, qos_esq);
        printf("[STATS] Observação: sistema soft real-time — deadlines perdidas resultam em parada segura, sem risco crítico.\n\n");
        vTaskDelay(xFrequency);
    }
}

static void CameraRouterTask(void *pvParameters) {
    RouterContext_t *ctx = (RouterContext_t *)pvParameters;
    Message_t message;

    if (ctx == NULL || ctx->source == NULL) {
        vTaskDelete(NULL);
    }

    printf("[ROUTER] Encaminhador iniciado para %lu destinos\n", (unsigned long)ctx->target_count);

    while (1) {
        if (xQueueReceive(ctx->source->tx_queue, &message, portMAX_DELAY) == pdTRUE) {
            for (size_t i = 0; i < ctx->target_count; ++i) {
                BaseNode_t *target = ctx->targets[i];
                if (target != NULL && target->rx_queue != NULL) {
                    if (xQueueSend(target->rx_queue, &message, pdMS_TO_TICKS(50)) != pdTRUE) {
                        printf("[ROUTER] ERRO: fila RX de %s cheia\n", target->node_id);
                    }
                }
            }
        }
    }
}

int main(void) {
    SetupConsoleUtf8();
    freertos_sim_init();

    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║   VALIDAÇÃO HOST - ESTEIRA INDUSTRIAL (SIMULAÇÃO POSIX)    ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");

    if (!SafetyNode_Init(&safety, "safety_master", 0)) {
        printf("[HOST] Falha ao inicializar SafetyNode\n");
        return -1;
    }

    if (!ConveyorNode_Init(&conveyor, "conveyor_belt", 0, 120)) {
        printf("[HOST] Falha ao inicializar ConveyorNode\n");
        return -1;
    }

    if (!PistonNode_CreatePistonA(&piston_a) || !PistonNode_CreatePistonB(&piston_b)) {
        printf("[HOST] Falha ao inicializar pistões\n");
        return -1;
    }

    uint16_t piston_ports[] = {5002, 5003};
    if (!CameraNode_Init(&camera, "camera_1", piston_ports, 2)) {
        printf("[HOST] Falha ao inicializar CameraNode\n");
        return -1;
    }

    SafetyNode_RegisterEmergencyCallback(&safety, HostEmergencyCallback);

    if (!SafetyNode_Start(&safety) ||
        !ConveyorNode_Start(&conveyor) ||
        !PistonNode_Start(&piston_a) ||
        !PistonNode_Start(&piston_b) ||
        !CameraNode_Start(&camera)) {
        printf("[HOST] Falha ao iniciar nós FreeRTOS simulados\n");
        return -1;
    }

    ConveyorNode_Run(&conveyor);

    xTaskCreate(BlinkTask, "Blink", 256, NULL, 1, NULL);
    xTaskCreate(SimulationTask, "Simulation", 1024, NULL, 2, NULL);
    xTaskCreate(StatsTask, "Stats", 2048, NULL, 1, NULL);
    camera_router_ctx.source = &camera.base;
    camera_router_ctx.targets[0] = &piston_a.base;
    camera_router_ctx.targets[1] = &piston_b.base;
    camera_router_ctx.target_count = 2;
    xTaskCreate(CameraRouterTask, "CamRouter", 512, &camera_router_ctx, 3, NULL);

    printf("[HOST] Scheduler POSIX ativo. Teste automático dura ~30 segundos.\n");
    vTaskDelay(pdMS_TO_TICKS(15000));

    printf("[HOST] Disparando emergência para validar parada geral.\n");
    safety.button_pressed = true;
    vTaskDelay(pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(10000));
    printf("[HOST] Simulação concluída. Finalize com Ctrl+C se desejar encerrar antes.\n");
    while (0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return 0;
}
