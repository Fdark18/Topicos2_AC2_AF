# Sistema de Esteira Industrial - STM32L053R8 Nucleo

## 📋 Visão Geral

Este projeto implementa um sistema de esteira industrial com classificação por cor usando **FreeRTOS** no **STM32L053R8 Nucleo board**.

### Hardware Necessário

- **STM32 Nucleo-L053R8**
- LEDs externos (opcional - a placa já tem LD2)
- Botão de emergência (usa o USER button da placa)
- Conexões para pistões (PA8, PA9)
- Cabo USB para programação e debug

### Especificações do STM32L053R8

- **MCU**: ARM Cortex-M0+ @ 32 MHz
- **Flash**: 64 KB
- **RAM**: 8 KB
- **Timers**: TIM2, TIM21, TIM22
- **USART**: 2x USART, 1x LPUART

## 🗂️ Estrutura do Projeto

```
mvp_freertos/
├── Core/
│   ├── Inc/                        # Headers
│   │   ├── stm32_config.h         # Configuração de hardware
│   │   ├── FreeRTOSConfig.h       # Configuração do FreeRTOS
│   │   ├── timing_config.h        # Configurações de tempo real
│   │   ├── base_node.h
│   │   ├── camera_node.h
│   │   ├── conveyor_node.h
│   │   ├── piston_node.h
│   │   ├── safety_node.h
│   │   ├── usart.h
│   │   └── gpio.h
│   └── Src/                        # Código fonte
│       ├── main_stm32.c           # Main adaptado para STM32
│       ├── base_node.c
│       ├── camera_node.c
│       ├── conveyor_node.c
│       ├── piston_node.c
│       ├── safety_node.c
│       ├── syscalls.c
│       ├── usart.c
│       └── gpio.c
├── Drivers/
│   └── STM32L0xx_HAL_Driver/      # HAL drivers (gerados pelo CubeMX)
├── Middlewares/
│   └── Third_Party/
│       └── FreeRTOS/              # FreeRTOS source code
└── README_STM32.md                # Este arquivo
```

## 🛠️ Configuração no STM32CubeIDE

### Passo 1: Criar Projeto no STM32CubeMX

1. Abra o **STM32CubeIDE**
2. Vá em **File > New > STM32 Project**
3. Selecione **Board Selector**
4. Procure por **NUCLEO-L053R8** e selecione
5. Clique em **Next**
6. Nome do projeto: `mvp_freertos`
7. Clique em **Finish**

### Passo 2: Configurar Periféricos no CubeMX

#### Clock Configuration
1. Na aba **Clock Configuration**:
   - Configure o **SYSCLK** para **32 MHz**
   - Source: **PLLCLK**
   - HSI: **16 MHz**
   - PLLMUL: **x4**
   - PLLDIV: **/2**

#### Pinout Configuration

**USART2** (Debug - já vem configurado para ST-Link):
- Mode: **Asynchronous**
- PA2: USART2_TX
- PA3: USART2_RX
- Baud Rate: 115200

**GPIO Outputs**:
- PA5: **GPIO_Output** (LD2 - LED de status)
- PB0: **GPIO_Output** (LED de erro)
- PA8: **GPIO_Output** (Pistão A)
- PA9: **GPIO_Output** (Pistão B)

**GPIO Input**:
- PC13: **GPIO_Input** (USER button - botão de emergência)

**TIM2** (PWM para motor):
- Channel 1: **PWM Generation CH1**
- PA0: TIM2_CH1
- Prescaler: 31 (32 MHz / 32 = 1 MHz)
- Counter Period: 999 (1 kHz PWM)

### Passo 3: Configurar FreeRTOS

1. Na aba **Middleware > FREERTOS**:
   - Interface: **CMSIS_V1**
   - Clique em **Enabled**

2. **Configuration**:
   - TOTAL_HEAP_SIZE: **4096** (4KB)
   - MAX_PRIORITIES: **6**
   - MINIMAL_STACK_SIZE: **64** words
   - USE_PREEMPTION: **Enabled**
   - USE_MUTEXES: **Enabled**
   - USE_COUNTING_SEMAPHORES: **Enabled**

3. **Tasks**:
   - Deixe vazio (vamos criar manualmente no código)

### Passo 4: Gerar Código

1. Clique em **Project > Generate Code**
2. O STM32CubeMX irá gerar a estrutura base

### Passo 5: Copiar Arquivos do Projeto

#### Copiar Headers para `Core/Inc/`:
```bash
cp stm32_config.h Core/Inc/
cp FreeRTOSConfig.h Core/Inc/
cp timing_config.h Core/Inc/
cp base_node.h Core/Inc/
cp camera_node.h Core/Inc/
cp conveyor_node.h Core/Inc/
cp piston_node.h Core/Inc/
cp safety_node.h Core/Inc/
cp usart.h Core/Inc/
cp gpio.h Core/Inc/
```

#### Copiar Sources para `Core/Src/`:
```bash
cp main_stm32.c Core/Src/main.c  # Substituir o main gerado
cp base_node.c Core/Src/
cp camera_node.c Core/Src/
cp conveyor_node.c Core/Src/
cp piston_node.c Core/Src/
cp safety_node.c Core/Src/
cp syscalls.c Core/Src/
cp usart.c Core/Src/
cp gpio.c Core/Src/
```

### Passo 6: Compilar

1. No STM32CubeIDE, clique em **Project > Build Project**
2. Verifique se não há erros de compilação
3. Se houver erros de memória, ajuste `configTOTAL_HEAP_SIZE` em [FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h)

### Passo 7: Descarregar no STM32

1. Conecte o Nucleo via USB
2. Clique em **Run > Debug** ou pressione **F11**
3. O código será gravado e o debug iniciado
4. Clique em **Resume** para executar

## 📊 Monitoramento via UART

### Configurar Terminal Serial

1. Abra um terminal serial (PuTTY, TeraTerm, CoolTerm, etc.)
2. Configure:
   - **Porta**: Detectar automaticamente (geralmente COMx no Windows)
   - **Baud Rate**: 115200
   - **Data Bits**: 8
   - **Stop Bits**: 1
   - **Parity**: None
   - **Flow Control**: None

### Saída Esperada

```
╔════════════════════════════════════════════════════════════╗
║   SISTEMA DE ESTEIRA INDUSTRIAL - STM32L053R8             ║
║   Sistema Distribuído de Tempo Real com FreeRTOS          ║
╚════════════════════════════════════════════════════════════╝

[INIT] Inicializando nós do sistema...
[MAIN] Inicializando Nó de Segurança (PRIORIDADE MÁXIMA)...
[safety_master] Nó de segurança inicializado
[MAIN] Inicializando Nó da Esteira...
[conveyor_belt] Nó da esteira inicializado
...

[SIMULACAO] Tarefa iniciada - Simulando detecções a cada 5s
[SIMULACAO] ===== Simulando detecção de VERDE =====
[camera_1] Detecção de cor: VERDE (500 pixels, confiança: 85.0%)
[piston_a] Ativação agendada para +2000ms
...
```

## ⚙️ Ajustes Importantes

### Limitações de RAM

O STM32L053R8 tem apenas **8KB de RAM**. Otimizações realizadas:

1. **Heap do FreeRTOS**: 4KB (50% da RAM)
2. **Stack das tasks**: Reduzido para mínimo necessário
3. **Buffers**: Minimizados
4. **Queues**: Tamanho reduzido de 10 para 5 elementos

### Se encontrar erro de falta de memória:

**Erro: "region RAM overflowed"**

Soluções:
1. Reduzir `configTOTAL_HEAP_SIZE` em [FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h)
2. Reduzir stack das tasks em [stm32_config.h](Core/Inc/stm32_config.h)
3. Desabilitar algumas features do FreeRTOS

**Erro: "Malloc failed"**

- Aumentar `configTOTAL_HEAP_SIZE` ligeiramente
- Verificar se não há memory leaks

## 🔧 Pinout do Nucleo-L053R8

### Conexões da Placa

| Pino   | Função              | Descrição                    |
|--------|---------------------|------------------------------|
| PA2    | USART2_TX           | Debug UART TX (ST-Link)      |
| PA3    | USART2_RX           | Debug UART RX (ST-Link)      |
| PA5    | GPIO_Output         | LED LD2 (Status - Verde)     |
| PA8    | GPIO_Output         | Pistão A (Verde→Direita)     |
| PA9    | GPIO_Output         | Pistão B (Azul→Esquerda)     |
| PA0    | TIM2_CH1            | PWM Motor da Esteira         |
| PB0    | GPIO_Output         | LED de Erro (Vermelho)       |
| PC13   | GPIO_Input          | USER Button (Emergência)     |

### Diagrama de Conexões Externas

```
                STM32 Nucleo-L053R8
                ┌─────────────────┐
                │                 │
    LED Status  │ PA5 (LD2)      │  (Onboard)
    LED Erro ───┤ PB0            │
    Pistão A ───┤ PA8            │
    Pistão B ───┤ PA9            │
    PWM Motor ──┤ PA0 (TIM2_CH1) │
    Botão Emerg─┤ PC13 (USER)    │  (Onboard)
                │                 │
    Debug TX ───┤ PA2 (USART2)   │  (ST-Link)
    Debug RX ───┤ PA3 (USART2)   │  (ST-Link)
                │                 │
                └─────────────────┘
```

## 📈 Prioridades das Tarefas

```
PRIORIDADE 5 (MÁXIMA) - Safety Monitor       (Thread Master)
PRIORIDADE 4          - Piston A/B Control   (Crítica)
PRIORIDADE 4          - Camera Processing    (Crítica)
PRIORIDADE 3          - Camera Capture       (Alta)
PRIORIDADE 2          - Conveyor Control     (Média)
PRIORIDADE 2          - Simulation           (Média)
PRIORIDADE 1          - Statistics/Blink     (Baixa)
PRIORIDADE 0          - Idle Task            (FreeRTOS)
```

## 🐛 Troubleshooting

### Problema: Código não compila

**Erro**: `undefined reference to 'printf'`

**Solução**: Certifique-se que `syscalls.c` está incluído no projeto.

---

**Erro**: `FreeRTOS.h: No such file or directory`

**Solução**:
1. Verifique se FreeRTOS foi habilitado no CubeMX
2. Gere código novamente
3. Verifique include paths no projeto

---

### Problema: Sistema trava ou reseta

**Causa**: Stack overflow ou falta de heap

**Solução**:
1. Habilite `configCHECK_FOR_STACK_OVERFLOW` em [FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h)
2. Aumente stack das tasks em [stm32_config.h](Core/Inc/stm32_config.h)
3. Monitore uso de heap com `xPortGetFreeHeapSize()`

---

### Problema: Printf não funciona

**Causa**: UART não configurado ou syscalls incorretos

**Solução**:
1. Verifique se USART2 está configurado em 115200 baud
2. Certifique-se que `syscalls.c` está compilado
3. Teste com HAL_UART_Transmit direto

---

### Problema: Tarefas não executam

**Causa**: Heap insuficiente para criar tasks

**Solução**:
1. Reduza `configTOTAL_HEAP_SIZE`
2. Reduza stack das tasks
3. Verifique `vApplicationMallocFailedHook`

## 📚 Documentação Adicional

- [README.md](README.md) - Documentação geral do projeto
- [DIAGRAMAS.md](DIAGRAMAS.md) - Diagramas de execução
- [CALCULO_TEMPO.md](CALCULO_TEMPO.md) - Análise de tempo real
- [STM32_SETUP_GUIDE.md](STM32_SETUP_GUIDE.md) - Guia detalhado de setup

## 🔗 Links Úteis

- [STM32L053R8 Datasheet](https://www.st.com/resource/en/datasheet/stm32l053r8.pdf)
- [Nucleo-L053R8 User Manual](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [STM32CubeIDE User Guide](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)

## ⚡ Próximos Passos

1. ✅ Compilar e testar no hardware
2. 🔲 Adicionar câmera real (ESP32-CAM via UART)
3. 🔲 Implementar comunicação I2C/SPI com sensores
4. 🔲 Otimizar consumo de energia (low-power modes)
5. 🔲 Adicionar watchdog timer
6. 🔲 Implementar log em Flash

---

**Desenvolvido para STM32L053R8 Nucleo + FreeRTOS**
