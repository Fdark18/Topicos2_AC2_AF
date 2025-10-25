# Guia de Configuração STM32CubeIDE

Este guia detalha o processo completo de configuração do projeto FreeRTOS para STM32 usando o STM32CubeIDE.

## Índice

1. [Requisitos](#requisitos)
2. [Criação do Projeto](#criação-do-projeto)
3. [Configuração de Periféricos](#configuração-de-periféricos)
4. [Configuração do FreeRTOS](#configuração-do-freertos)
5. [Importação dos Arquivos](#importação-dos-arquivos)
6. [Compilação e Gravação](#compilação-e-gravação)
7. [Debug e Monitoramento](#debug-e-monitoramento)
8. [Troubleshooting](#troubleshooting)

---

## Requisitos

### Hardware

- **Microcontrolador STM32**: F4xx, F7xx ou H7xx (recomendado: STM32F407VG Discovery Board)
- **Programador/Debugger**: ST-Link V2 (integrado na Discovery Board)
- **Componentes do Sistema**:
  - Botão de emergência (push button)
  - Motor DC com driver PWM
  - 2x Pistões pneumáticos/solenoides
  - 2x LEDs (status e erro)
  - Fonte de alimentação adequada

### Software

- **STM32CubeIDE** v1.12.0 ou superior
- **STM32CubeMX** (integrado ao CubeIDE)
- **Driver ST-Link** (instalado com o CubeIDE)
- **Terminal Serial**: PuTTY, TeraTerm ou Serial Monitor do Arduino IDE

---

## Criação do Projeto

### Passo 1: Criar Novo Projeto STM32

1. Abra o **STM32CubeIDE**
2. Vá em `File → New → STM32 Project`
3. Na aba **Board Selector**:
   - Procure por `STM32F407VG` (ou seu MCU específico)
   - Selecione a Discovery Board
   - Clique em `Next`
4. Configure o projeto:
   - **Project Name**: `mvp_freertos_stm32`
   - **Targeted Language**: `C`
   - **Targeted Binary Type**: `Executable`
   - **Targeted Project Type**: `STM32Cube`
5. Clique em `Finish`
6. Quando perguntado sobre inicializar periféricos, selecione `Yes`

### Passo 2: Configurar Clock

1. Na aba **Pinout & Configuration**, vá para **Clock Configuration**
2. Configure para máxima frequência:
   - **STM32F407VG**: 168 MHz
   - **Input frequency**: 8 MHz (HSE do cristal externo)
   - **PLL Source**: HSE
   - **PLLM**: 8
   - **PLLN**: 336
   - **PLLP**: 2
   - **System Clock Mux**: PLLCLK
3. Verifique os clocks dos barramentos:
   - **AHB Clock**: 168 MHz
   - **APB1 Clock**: 42 MHz (APB1 Timer: 84 MHz)
   - **APB2 Clock**: 84 MHz (APB2 Timer: 168 MHz)

---

## Configuração de Periféricos

### Passo 3: Configurar GPIO

#### Botão de Emergência (PA0)

1. Clique em `PA0` no diagrama de pinos
2. Selecione `GPIO_Input`
3. Em **GPIO Settings**:
   - **GPIO mode**: Input mode
   - **GPIO Pull-up/Pull-down**: Pull-up
   - **User Label**: `EMERGENCY_BTN`
4. Em **NVIC Settings** (para interrupção - opcional):
   - Habilite `EXTI line0 interrupt`
   - Priority: `5`

#### Pistões (PC0, PC1)

1. Clique em `PC0` → `GPIO_Output` → Label: `PISTON_A`
2. Clique em `PC1` → `GPIO_Output` → Label: `PISTON_B`
3. Configure ambos:
   - **GPIO mode**: Output Push Pull
   - **GPIO Pull-up/Pull-down**: No pull-up and no pull-down
   - **Maximum output speed**: High
   - **Initial Level**: Low

#### LEDs de Status (PA5, PA6)

1. Clique em `PA5` → `GPIO_Output` → Label: `LED_STATUS`
2. Clique em `PA6` → `GPIO_Output` → Label: `LED_ERROR`
3. Configure ambos:
   - **GPIO mode**: Output Push Pull
   - **Maximum output speed**: Low
   - **Initial Level**: Low

#### Trigger da Câmera (PB7)

1. Clique em `PB7` → `GPIO_Output` → Label: `CAMERA_TRIGGER`

### Passo 4: Configurar PWM para Motor (TIM4)

1. Em **Timers**, expanda `TIM4`
2. Configure:
   - **Clock Source**: Internal Clock
   - **Channel1**: PWM Generation CH1
3. Clique em `PB6` no diagrama → `TIM4_CH1`
4. Em **Parameter Settings** do TIM4:
   - **Prescaler**: `83` (para clock de 1 MHz)
   - **Counter Mode**: Up
   - **Counter Period (AutoReload)**: `999` (para PWM de 1 kHz)
   - **Internal Clock Division**: No Division
   - **PWM Mode**: PWM mode 1
   - **Pulse (CH1)**: `0` (duty cycle inicial 0%)
   - **CH Polarity**: High

### Passo 5: Configurar UART

#### USART2 (Debug/Printf)

1. Em **Connectivity**, expanda `USART2`
2. Configure:
   - **Mode**: Asynchronous
   - **Baud Rate**: `115200`
   - **Word Length**: 8 Bits
   - **Parity**: None
   - **Stop Bits**: 1
   - **Hardware Flow Control**: Disable
3. Pinos serão automaticamente mapeados:
   - `PA2` → USART2_TX
   - `PA3` → USART2_RX

#### USART1 (Comunicação - Opcional)

1. Expanda `USART1`
2. Configure com mesmos parâmetros do USART2
3. Pinos:
   - `PA9` → USART1_TX
   - `PA10` → USART1_RX

### Passo 6: Configurar DMA (Opcional - para UART)

Se desejar usar DMA para UART:

1. Em `USART2`, aba **DMA Settings**
2. Clique em `Add`
3. Selecione:
   - **DMA Request**: USART2_TX
   - **Stream**: DMA1 Stream 6
   - **Direction**: Memory to Peripheral
   - **Priority**: Low
   - **Mode**: Normal
4. Repita para USART2_RX se necessário

---

## Configuração do FreeRTOS

### Passo 7: Habilitar FreeRTOS

1. Em **Middleware**, expanda `FREERTOS`
2. **Interface**: `CMSIS_V2` (recomendado)
3. Habilite FreeRTOS clicando em `Enabled`

### Passo 8: Configurar FreeRTOS

#### Config Parameters

1. Vá para **Config parameters**
2. Configure:
   - **TICK_RATE_HZ**: `1000` (tick de 1 ms)
   - **MAX_PRIORITIES**: `11` (0-10)
   - **MINIMAL_STACK_SIZE**: `128` words
   - **TOTAL_HEAP_SIZE**: `40960` (40 KB)
   - **USE_PREEMPTION**: Enabled
   - **USE_TIME_SLICING**: Enabled
   - **USE_IDLE_HOOK**: Disabled (ou Enabled se precisar)
   - **USE_TICK_HOOK**: Disabled
   - **USE_MALLOC_FAILED_HOOK**: Enabled
   - **CHECK_FOR_STACK_OVERFLOW**: Option2
   - **HEAP_SCHEME**: heap_4 (recomendado)

#### Incluir Módulos

3. Em **Include parameters**:
   - **vTaskDelay**: Enabled
   - **vTaskDelayUntil**: Enabled
   - **xTaskGetTickCount**: Enabled
   - **uxTaskPriorityGet**: Enabled
   - **vTaskPrioritySet**: Enabled
   - **vTaskSuspend**: Enabled

### Passo 9: Criar Tasks Iniciais

1. Vá para **Tasks and Queues**
2. Crie as seguintes tasks:

| Task Name | Priority | Stack Size (Words) | Entry Function |
|-----------|----------|-------------------|----------------|
| SafetyTask | 10 (osPriorityRealtime) | 256 | SafetyTask |
| CameraTask | 4 (osPriorityAboveNormal) | 512 | CameraTask |
| PistonATask | 4 (osPriorityAboveNormal) | 256 | PistonATask |
| PistonBTask | 4 (osPriorityAboveNormal) | 256 | PistonBTask |
| ConveyorTask | 2 (osPriorityNormal) | 256 | ConveyorTask |
| MonitorTask | 1 (osPriorityBelowNormal) | 512 | MonitorTask |

3. Crie as filas (Queues):

| Queue Name | Size | Item Size | Type |
|------------|------|-----------|------|
| ColorEventQueue | 10 | sizeof(ColorEvent) | osMessageQueue |
| CommRxQueue | 10 | 32 bytes | osMessageQueue |

---

## Importação dos Arquivos

### Passo 10: Adicionar Arquivos Fonte

1. No **Project Explorer**, localize a pasta `Core/Src`
2. Copie os seguintes arquivos para esta pasta:
   - `syscalls.c`
   - `usart.c`
   - `gpio.c`
   - `base_node.c`
   - `safety_node.c`
   - `camera_node.c`
   - `piston_node.c`
   - `conveyor_node.c`

3. Localize a pasta `Core/Inc`
4. Copie os headers:
   - `usart.h`
   - `gpio.h`
   - `stm32_config.h`
   - `timing_config.h`
   - `base_node.h`
   - `safety_node.h`
   - `camera_node.h`
   - `piston_node.h`
   - `conveyor_node.h`

### Passo 11: Configurar Include Paths

1. Clique com botão direito no projeto → `Properties`
2. Vá para `C/C++ Build → Settings`
3. Em `MCU GCC Compiler → Include paths`, adicione:
   ```
   ../Core/Inc
   ../Middlewares/Third_Party/FreeRTOS/Source/include
   ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
   ```

### Passo 12: Modificar main.c

Adicione no início do `main.c` (USER CODE BEGIN Includes):

```c
/* USER CODE BEGIN Includes */
#include "stm32_config.h"
#include "gpio.h"
#include "usart.h"
#include "safety_node.h"
#include "camera_node.h"
#include "piston_node.h"
#include "conveyor_node.h"
/* USER CODE END Includes */
```

No `main()`, após `MX_GPIO_Init()`:

```c
/* USER CODE BEGIN 2 */

/* Initialize custom peripherals */
GPIO_InitAll(NULL);  // Initialize GPIO with default config

/* Initialize UART for printf */
USART_Config_t uart_config = {
    .huart = &huart2,
    .baudrate = 115200,
    .tx_buffer_size = 512,
    .rx_buffer_size = 512,
    .use_dma = false
};
USART_Init(&uart_config);

printf("\r\n=== FreeRTOS Conveyor Belt System ===\r\n");
printf("System Clock: %lu MHz\r\n", HAL_RCC_GetHCLKFreq() / 1000000);

/* USER CODE END 2 */
```

---

## Compilação e Gravação

### Passo 13: Compilar o Projeto

1. Clique em `Project → Build All` (ou pressione `Ctrl+B`)
2. Verifique a aba **Console** para erros
3. Se houver erros de compilação:
   - Verifique os include paths
   - Confirme que todos os arquivos foram copiados
   - Ajuste o arquivo HAL para seu MCU específico

### Passo 14: Configurar Debug

1. Clique em `Run → Debug Configurations`
2. Crie uma nova `STM32 Cortex-M C/C++ Application`
3. Em **Debugger**:
   - **Debug probe**: ST-LINK (ST-LINK GDB server)
   - **Interface**: SWD
   - **Reset Mode**: Software system reset
4. Em **Startup**:
   - Marque `Enable semihosting`
   - Marque `Set breakpoint at: main`

### Passo 15: Gravar e Executar

1. Conecte a placa STM32 via USB
2. Clique em `Run → Debug` (ou pressione `F11`)
3. O programa será gravado e iniciará automaticamente
4. Use os controles de debug para:
   - Pausar/continuar execução
   - Inserir breakpoints
   - Inspecionar variáveis
   - Ver registradores

---

## Debug e Monitoramento

### Passo 16: Monitor Serial (Printf)

1. Abra um terminal serial (PuTTY, TeraTerm, etc.)
2. Configure:
   - **Port**: Identifique a porta COM do ST-Link Virtual COM Port
   - **Baud rate**: 115200
   - **Data bits**: 8
   - **Stop bits**: 1
   - **Parity**: None
   - **Flow control**: None
3. Conecte e você verá as mensagens de debug do sistema

### Passo 17: Live Expressions

Para monitorar variáveis em tempo real:

1. No modo Debug, abra `Window → Show View → Live Expressions`
2. Adicione expressões para monitorar:
   - Contadores de tasks
   - Estados dos pistões
   - Métricas de latência
   - Status de emergência

### Passo 18: FreeRTOS Task List

1. No modo Debug, abra `Window → Show View → FreeRTOS → Task List`
2. Você verá:
   - Todas as tasks criadas
   - Stack usage de cada task
   - Estado atual (Running/Ready/Blocked)
   - Prioridades

---

## Troubleshooting

### Problema: Projeto não compila

**Erro**: `fatal error: stm32f4xx_hal.h: No such file or directory`

**Solução**:
1. Verifique se gerou o código com CubeMX
2. Confirme include paths em Project Properties
3. Altere `#include "stm32f4xx_hal.h"` para corresponder ao seu MCU

### Problema: HardFault ao iniciar FreeRTOS

**Erro**: Sistema trava logo após `osKernelStart()`

**Solução**:
1. Aumente `configTOTAL_HEAP_SIZE` em FreeRTOSConfig.h
2. Verifique stack sizes das tasks
3. Habilite `configCHECK_FOR_STACK_OVERFLOW` para detectar overflow

### Problema: Printf não funciona

**Erro**: Nenhuma mensagem no terminal serial

**Solução**:
1. Verifique se `syscalls.c` está incluído no projeto
2. Confirme configuração da UART (baudrate, pinos)
3. Habilite semihosting nas configurações de debug
4. Teste com `HAL_UART_Transmit()` diretamente

### Problema: PWM não gera sinal

**Erro**: Motor não liga

**Solução**:
1. Verifique se chamou `HAL_TIM_PWM_Start()` após inicialização
2. Confirme configuração do timer (prescaler, period)
3. Use osciloscópio para verificar sinal no pino PB6
4. Ajuste duty cycle para valor > 0

### Problema: Tasks não executam

**Erro**: Apenas DefaultTask roda

**Solução**:
1. Verifique se criou as tasks antes de `osKernelStart()`
2. Confirme prioridades (não podem todas ser 0)
3. Verifique se há `vTaskDelay()` nas tasks para liberar CPU
4. Use FreeRTOS Task List para diagnóstico

### Problema: ST-Link não detectado

**Erro**: `No ST-LINK detected`

**Solução**:
1. Reinstale drivers ST-Link
2. Troque cabo USB
3. Verifique jumpers da placa (BOOT0/BOOT1)
4. Use ST-Link Utility para atualizar firmware do ST-Link

---

## Próximos Passos

Após configuração bem-sucedida:

1. **Testar Hardware**:
   - Verifique cada pino GPIO com multímetro/osciloscópio
   - Teste botão de emergência
   - Confirme acionamento dos pistões

2. **Calibrar Timings**:
   - Ajuste velocidade da esteira em `timing_config.h`
   - Meça distância câmera-pistão
   - Recalcule tempos de viagem

3. **Integrar Câmera**:
   - Conecte câmera via UART ou SPI
   - Implemente protocolo de comunicação
   - Teste detecção de cores

4. **Validar Tempo Real**:
   - Use osciloscópio para medir jitter
   - Valide deadlines com timestamps
   - Calcule WCET de cada task

5. **Otimizar Performance**:
   - Habilite DMA para UART se necessário
   - Ajuste prioridades se houver deadline misses
   - Profile com ferramentas de análise

---

## Recursos Adicionais

- [STM32CubeIDE User Guide](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [HAL Driver User Manual](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)

---

## Contato e Suporte

Para dúvidas ou problemas específicos do projeto:
- Consulte a documentação em `README.md`
- Veja diagramas de execução em `DIAGRAMAS.md`
- Análise de timing em `CALCULO_TEMPO.md`

**Desenvolvido para o curso de Tópicos Especiais em Sistemas Distribuídos em Tempo Real**
