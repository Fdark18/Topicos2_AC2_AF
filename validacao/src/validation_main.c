#include "freertos_sim.h"

#include "camera_node.h"
#include "conveyor_node.h"
#include "piston_node.h"
#include "safety_node.h"

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

static SafetyNode_t safety;
static ConveyorNode_t conveyor;
static CameraNode_t camera;
static PistonNode_t piston_a;
static PistonNode_t piston_b;

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
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);
    uint8_t sequence = 0;

    printf("[SIMULACAO] Tarefa iniciada - alternando cores a cada 5s\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        if (!SafetyNode_GlobalEmergencyCheck()) {
            if ((sequence % 2) == 0) {
                printf("[SIMULACAO] >>> DETECCAO ARTIFICIAL VERDE\n");
                CameraNode_SimulateDetection(&camera, COLOR_GREEN, 520, 0.92f);
            } else {
                printf("[SIMULACAO] >>> DETECCAO ARTIFICIAL AZUL\n");
                CameraNode_SimulateDetection(&camera, COLOR_BLUE, 480, 0.88f);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
            CameraNode_SimulateObjectExit(&camera);
            sequence++;
        } else {
            printf("[SIMULACAO] Sistema em emergência - aguardando reset...\n");
        }

        vTaskDelay(xFrequency);
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
        vTaskDelay(xFrequency);
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

    printf("[HOST] Scheduler POSIX ativo. Teste automático dura ~30 segundos.\n");
    vTaskDelay(pdMS_TO_TICKS(15000));

    printf("[HOST] Disparando emergência para validar parada geral.\n");
    safety.button_pressed = true;
    vTaskDelay(pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(10000));
    printf("[HOST] Simulação concluída. Finalize com Ctrl+C se desejar encerrar antes.\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    return 0;
}
