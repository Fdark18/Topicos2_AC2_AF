# MVP FreeRTOS - Sistema de Esteira Industrial

## Visão Geral

Implementação em **C + FreeRTOS** do sistema de classificação automática por cor com **Thread Master de Emergência**. Este projeto demonstra como construir sistemas distribuídos de tempo real em hardware embarcado com segurança crítica.

###  **DESTAQUE: Thread Master de Emergência**
O sistema possui uma **thread de emergência com prioridade MÁXIMA (10)** que fica logo abaixo do escalonador FreeRTOS. Quando o botão vermelho de segurança é acionado, esta thread para TODAS as operações (esteira, câmera, pistões) em menos de 10ms!

## Arquitetura com Thread Master

```
                      ╔═══════════════════════╗
                      ║   SAFETY MONITOR      ║ ← THREAD MASTER (P:10)
                      ║   Botão Emergência    ║   Embaixo do escalonador
                      ║   PARA TUDO quando    ║   Máxima prioridade
                      ║   botão pressionado   ║
                      ╚═══════════╤═══════════╝
                                  │ Monitora continuamente
                      ┌───────────┼───────────┬───────────┐
                      │           │           │           │
         ┌────────────▼────┐  ┌──▼────┐  ┌───▼──────┐  ┌─▼────────┐
         │  ConveyorNode   │  │Camera │  │ Piston A │  │ Piston B │
         │  (Esteira P:2)  │  │(P:4)  │  │  (P:4)   │  │  (P:4)   │
         │                 │  │       │  │          │  │          │
         │ • Motor esteira │  │Detecta│  │Verde→Dir.│  │Azul→Esq. │
         │ • Controla vel. │  │ cores │  │          │  │          │
         └─────────────────┘  └───┬───┘  └────▲─────┘  └────▲─────┘
                                  │           │             │
                                  │  ColorDetectionEvent    │
                                  │  (via FreeRTOS Queue)   │
                                  └───────────┴─────────────┘

         Todas as threads verificam flag de emergência:
         if (SafetyNode_GlobalEmergencyCheck()) { PARAR(); }
```

## Estrutura de Arquivos

```
mvp_freertos/
├── 📄 README.md                    # Este arquivo (documentação completa)
├── 📄 DIAGRAMAS.md                 #  Diagramas de execução das threads
├── 📄 CALCULO_TEMPO.md             #  Cálculos de WCET e schedulability
├── 📄 timing_config.h              # Configurações de tempo real e prioridades
│
├── 🔧 main.c                       #  Programa principal (integra tudo)
├── 🔧 base_node.h/c                # Classe base para todos os nós
│
├── 🟢 safety_node.h/c              #  Nó de emergência (Thread Master P:10)
├── 🔵 conveyor_node.h/c            #  Nó da esteira (Thread P:2)
├── 📹 camera_node.h/c              # Nó da câmera (Thread P:4)
├── 🔧 piston_node.h/c              # Nós dos pistões (Threads P:4)
```

### Arquivos Principais

- ** DIAGRAMAS.md**: Diagramas visuais completos da execução das threads
- ** CALCULO_TEMPO.md**: Cálculos detalhados de WCET, utilização de CPU e schedulability
- ** safety_node.c**: Implementação da Thread Master de emergência
- ** main.c**: Integração de todos os nós com ordem correta de inicialização
- ** timing_config.h**: Todas as configurações de prioridades e timing

## Componentes (Threads/Nós)

### 0. ⚠️ **SafetyNode (safety_node.h/c)** - Thread Master
**PRIORIDADE MÁXIMA (10) - Embaixo apenas do escalonador**
- 🚨 Monitora botão de emergência continuamente (polling 10ms)
- 🛑 Para TUDO imediatamente quando acionado (< 10ms)
- 🔴 Seta flag global `g_emergency_stop = true`
-  Preempta qualquer outra thread do sistema
-  Estatísticas de acionamentos de emergência
-  Reset seguro após correção do problema

**Como funciona:**
```c
// Thread Master monitora botão
while(1) {
    if (button_pressed) {
        g_emergency_stop = true;  // Para TUDO
        notify_all_nodes();
    }
    vTaskDelay(10ms);  // Alta frequência
}

// Todas as outras threads verificam
while(1) {
    if (SafetyNode_GlobalEmergencyCheck()) {
        break;  // Para operação
    }
    // ... operação normal
}
```

### 1. BaseNode (base_node.h/c)
Classe base que fornece funcionalidades comuns:
-  Gerenciamento de tarefas FreeRTOS
-  Filas de comunicação (RX/TX)
-  Cálculo de métricas de tempo real
-  Sistema de heartbeat
-  Handlers customizáveis para mensagens

### 2. ConveyorNode (conveyor_node.h/c) - Nó da Esteira
**Thread Prioridade 2**
- 🔵 Controla motor da esteira transportadora
- ⚙️ Velocidade configurável (padrão: 100 mm/s)
- 🛑 Para imediatamente em caso de emergência
- 📊 Estatísticas: objetos transportados, paradas, runtime
- 🔧 Controle PWM do motor (GPIO)

### 3. CameraNode (camera_node.h/c) - Nó da Câmera
**Thread Prioridade 4 (Crítica)**
- 📹 Captura e processamento de imagens
- 🎨 Detecção de cores (azul/verde)
- 📊 Validação de qualidade
- 📡 Broadcast de eventos para pistões
- 📈 Estatísticas de detecção

### 4. PistonNode (piston_node.h/c) - Nós dos Pistões
**Threads Prioridade 4 (Críticas)**
- 🔧 **Pistão A**: Verde → Direita
- 🔧 **Pistão B**: Azul → Esquerda
- ⏱️ Agendamento de ativações com delay calculado
- 🎯 Controle GPIO para atuadores
- 📊 Métricas de deadline e QoS
- 🛑 Para ativações pendentes em emergência

### 5. Main (main.c) - Programa Principal
Integra todos os componentes:
- 🚀 Inicialização em ordem correta (Safety → Conveyor → Pistões → Câmera)
- 🔄 Criação de tarefas auxiliares
- 🎲 Simulação de detecções (para teste)
- 📈 Relatório periódico de estatísticas

## Compilação

### Para ESP32 (ESP-IDF)

```bash
# Configure o projeto
idf.py menuconfig

# Compile
idf.py build

# Flash no ESP32
idf.py flash

# Monitor serial
idf.py monitor
```

### Para outras plataformas

Ajuste o arquivo de configuração do FreeRTOS (`FreeRTOSConfig.h`) e compile com seu toolchain:

```bash
# Exemplo com GCC
gcc -o sistema_esteira main.c base_node.c camera_node.c piston_node.c \
    -I. -I/path/to/freertos/include \
    -L/path/to/freertos/lib -lfreertos -lpthread
```

## Configuração

### FreeRTOSConfig.h (principais parâmetros)

```c
#define configUSE_PREEMPTION              1
#define configCPU_CLOCK_HZ                160000000  // ESP32: 160MHz
#define configTICK_RATE_HZ                1000       // 1ms tick
#define configMAX_PRIORITIES              5
#define configMINIMAL_STACK_SIZE          128
#define configTOTAL_HEAP_SIZE             (80 * 1024)

// Tempo real
#define configUSE_TIME_SLICING            0
#define configUSE_TIMERS                  1
```

### Parâmetros do Sistema

Em `base_node.h`:

```c
#define DEADLINE_US 50000  // 50ms deadline
```

Em `base_node.c` (construtor):

```c
node->belt_speed_mm_s = 100;                  // Velocidade da esteira
node->camera_to_piston_distance_mm = 200;     // Distância câmera-pistão
```

## Uso

### Exemplo Básico

```c
#include "camera_node.h"
#include "piston_node.h"

void app_main(void) {
    CameraNode_t camera;
    PistonNode_t piston_a, piston_b;

    // Inicializa
    uint16_t ports[] = {5002, 5003};
    CameraNode_Init(&camera, "camera_1", ports, 2);
    PistonNode_CreatePistonA(&piston_a);
    PistonNode_CreatePistonB(&piston_b);

    // Inicia
    CameraNode_Start(&camera);
    PistonNode_Start(&piston_a);
    PistonNode_Start(&piston_b);

    // Simula detecção
    CameraNode_SimulateDetection(&camera, COLOR_GREEN, 500, 0.85f);
}
```

### Integração com Câmera Real

Para usar com ESP32-CAM:

```c
#include "esp_camera.h"

// Configure câmera
camera_config_t config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    // ... mais configurações
};

esp_camera_init(&config);

// No loop de processamento
camera_fb_t *fb = esp_camera_fb_get();
Color_t detected = CameraNode_ProcessFrame(&camera, fb->buf, fb->len);
esp_camera_fb_return(fb);
```

### Controle de GPIO para Pistões

```c
#include "driver/gpio.h"

#define PISTON_A_GPIO 25
#define PISTON_B_GPIO 26

void PistonNode_PhysicalActivation(PistonNode_t *piston) {
    uint8_t pin = (piston->target_color == COLOR_GREEN) ?
                  PISTON_A_GPIO : PISTON_B_GPIO;

    gpio_set_level(pin, 1);        // Ativa
    vTaskDelay(pdMS_TO_TICKS(50)); // 50ms
    gpio_set_level(pin, 0);        // Desativa
}
```

## Métricas de Tempo Real

O sistema monitora:

- **Latência Total**: Tempo entre detecção e ativação
- **Deadline**: Prazo máximo (50ms padrão)
- **Jitter**: Variação entre ativações
- **QoS Score**: Qualidade do serviço (0.0 a 1.0)

### Exemplo de Saída

```
[piston_a] METRICAS DE TEMPO REAL:
[piston_a]   - Latencia total: 2045 ms
[piston_a]   - Deadline: 50 ms [OK CUMPRIDO]
[piston_a]   - Jitter: 12 ms
[piston_a]   - QoS Score: 95.4%
[piston_a]   - Qualidade detecção: 85.0%
```

## Hierarquia de Prioridades das Threads

```
ESCALONADOR FREERTOS
════════════════════════════════════════
          ↓
10 - Safety Monitor       (THREAD MASTER - Emergência)
          ↓ Pode preemptar TODAS as outras
─────────────────────────────────────────
 4 - Piston A Control     (Crítica - tempo real)
 4 - Piston B Control     (Crítica - tempo real)
 4 - Camera Processing    (Crítica - detecção)
─────────────────────────────────────────
 3 - Camera Capture       (Importante)
 3 - Communication RX     (Importante)
─────────────────────────────────────────
 2 - Conveyor Control     (Regular - esteira)
 2 - Communication TX     (Regular)
 2 - Heartbeat            (Regular)
 2 - Simulation           (Teste)
─────────────────────────────────────────
 1 - Monitor              (Background)
 1 - Statistics           (Background)
─────────────────────────────────────────
 0 - Idle Task            (FreeRTOS automático)
```

### Importância da Thread Master (P:10)
- Fica **embaixo apenas do escalonador**
- Quando botão de emergência é acionado, **interrompe tudo imediatamente**
- Threads de pistão (P:4) param em ~6ms
- Esteira (P:2) para em ~101ms
- **Sistema seguro em < 10ms**

## Diferenças do MVP Python

| Aspecto | MVP Python | MVP FreeRTOS |
|---------|-----------|--------------|
| **Linguagem** | Python | C |
| **SO** | Python threading | FreeRTOS |
| **Comunicação** | UDP sockets | Queues FreeRTOS |
| **Latência** | ~10-50ms | ~1-5ms |
| **Determinismo** | Baixo | Alto |
| **Consumo** | Alto | Muito baixo |
| **Hardware** | PC/Laptop | Microcontrolador |
| **Visão** | OpenCV | ESP-WHO/Simplificada |

## Vantagens da Implementação FreeRTOS

 **Tempo Real Garantido**: Deadlines determinísticos
 **Baixa Latência**: Resposta em microssegundos
 **Eficiência Energética**: Ideal para embarcados
 **Confiabilidade**: Sem overhead de SO completo
 **Custo**: Hardware mais barato (ESP32 ~$5)

## Limitações

 **Memória Restrita**: ~80KB heap vs GB em PC
 **Processamento Limitado**: Imagens menores/simplificadas
 **Debugging**: Mais complexo sem printf robusto
 **Networking**: Requer stack TCP/IP adicional

## Próximos Passos

1. **Integrar ESP32-CAM**: Captura real de imagens
2. **Implementar Networking**: UDP sobre Wi-Fi (lwIP)
3. **Adicionar Monitor**: Nó para visualização de métricas
4. **Otimizar Visão**: Algoritmos mais eficientes
5. **Tolerância a Falhas**: Watchdog e recovery

## Troubleshooting

### Problema: Deadlines sendo perdidos

**Sintomas**: QoS < 0.60, latências > 1500ms

**Soluções**:
```c
// Verificar stack usage
UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(task_handle);
printf("Stack livre: %u words\n", highWaterMark);

// Evitar printf() em tarefas críticas (usar logging assíncrono)
// Otimizar algoritmo de visão computacional
```

### Problema: Sistema trava ou reinicia

**Sintomas**: Watchdog reset, sistema congela

**Soluções**:
```c
// 1. Aumentar stack das tarefas
#define STACK_SIZE_CAMERA_CAPTURE    8192  // Era 4096

// 2. SEMPRE adicionar delays em loops
while(1) {
    // processamento...
    vTaskDelay(pdMS_TO_TICKS(10));  // OBRIGATÓRIO!
}

// 3. Monitorar heap
size_t free_heap = xPortGetFreeHeapSize();
printf("Heap livre: %u bytes\n", free_heap);
```

### Problema: Botão de emergência não responde rápido

**Causa**: Prioridade da Safety Thread muito baixa

**Solução**:
```c
// Em timing_config.h
#define PRIORITY_EMERGENCY    10  // DEVE ser a mais alta!
```

---

## Recursos e Documentação

### Documentação do Projeto
- **[DIAGRAMAS.md](DIAGRAMAS.md)**: Diagramas completos de execução das threads
- **[CALCULO_TEMPO.md](CALCULO_TEMPO.md)**: Análise detalhada de WCET, schedulability e otimizações
- **[timing_config.h](timing_config.h)**: Configurações de prioridades e timing

### FreeRTOS
- **Site Oficial**: https://www.freertos.org
- **Documentação**: https://www.freertos.org/Documentation/RTOS_book.html
- **GitHub**: https://github.com/FreeRTOS/FreeRTOS
- **Tutorial**: https://www.freertos.org/FreeRTOS-quick-start-guide.html

### ESP32
- **ESP-IDF**: https://docs.espressif.com/projects/esp-idf
- **FreeRTOS no ESP32**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
- **ESP32-CAM**: https://randomnerdtutorials.com/esp32-cam-video-streaming-face-recognition-arduino-ide/

### Projeto Original
- **MVP Python**: `../mvp/` - Versão Python do sistema

---

## Ordem de Leitura Recomendada

Para entender o sistema completamente:

1. **README.md** (este arquivo) - Visão geral e conceitos
2. **[DIAGRAMAS.md](DIAGRAMAS.md)** - Visualização da execução
3. **[CALCULO_TEMPO.md](CALCULO_TEMPO.md)** - ⭐ Análise de tempo e schedulability
4. **[main.c](main.c)** - Código de integração
5. **[safety_node.c](safety_node.c)** - Thread Master de emergência
6. **[timing_config.h](timing_config.h)** - Configurações de sistema

---

## Licença

Este projeto é fornecido como material educacional para estudo de sistemas distribuídos de tempo real.

---

**Desenvolvido para disciplina de Tópicos Especiais em Sistemas Distribuídos**
