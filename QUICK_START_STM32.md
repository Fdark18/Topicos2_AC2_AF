# 🚀 Quick Start - STM32L053R8 Nucleo

## Guia Rápido de 5 Minutos

### ✅ Pré-requisitos

- [ ] **STM32CubeIDE** instalado ([Download aqui](https://www.st.com/en/development-tools/stm32cubeide.html))
- [ ] **STM32 Nucleo-L053R8** board
- [ ] Cabo USB tipo Mini-B

### 📦 Passo 1: Criar Projeto Novo

1. Abra **STM32CubeIDE**
2. **File > New > STM32 Project**
3. Aba **Board Selector**
4. Digite: `NUCLEO-L053R8`
5. Selecione a placa e clique **Next**
6. Nome: `mvp_freertos`
7. **Finish**

### ⚙️ Passo 2: Configurar no STM32CubeMX

A janela do CubeMX abrirá automaticamente.

#### 2.1 - Clock Configuration

1. Clique na aba **Clock Configuration**
2. Configure:
   - Input frequency: **16 MHz** (HSI)
   - PLLMUL: **x4**
   - PLLDIV: **/2**
   - HCLK: **32 MHz** ✓

#### 2.2 - Pinout Configuration

**USART2** (já configurado):
- ✓ PA2: USART2_TX
- ✓ PA3: USART2_RX

**GPIO Outputs** (clique nos pinos no diagrama):
- **PA5**: `GPIO_Output` (renomeie: `LED_STATUS`)
- **PA8**: `GPIO_Output` (renomeie: `PISTON_A`)
- **PA9**: `GPIO_Output` (renomeie: `PISTON_B`)
- **PB0**: `GPIO_Output` (renomeie: `LED_ERROR`)

**GPIO Input**:
- **PC13**: Já está como `GPIO_Input` (USER button)

**TIM2** (PWM):
1. Na árvore à esquerda: **Timers > TIM2**
2. **Clock Source**: `Internal Clock`
3. **Channel1**: `PWM Generation CH1`
4. Na aba **Configuration**:
   - **Prescaler**: `31`
   - **Counter Period**: `999`

#### 2.3 - FreeRTOS

1. **Middleware > FREERTOS**
2. Marque **Enabled**
3. **Interface**: `CMSIS_V1`
4. Na aba **Config parameters**:
   - `TOTAL_HEAP_SIZE`: **4096**
   - `MAX_PRIORITIES`: **6**

### 💾 Passo 3: Gerar Código

1. **Project > Generate Code**
2. Se perguntar sobre perspectiva, clique **Yes**
3. Aguarde a geração

### 📁 Passo 4: Copiar Arquivos do Projeto

#### Navegue até a pasta do projeto gerado:
```
Workspace/mvp_freertos/
```

#### Copie os arquivos:

**Headers** (para `Core/Inc/`):
```
stm32_config.h
FreeRTOSConfig.h (substituir)
timing_config.h
base_node.h
camera_node.h
conveyor_node.h
piston_node.h
safety_node.h
usart.h
gpio.h
```

**Source** (para `Core/Src/`):
```
main_stm32.c → main.c (SUBSTITUIR o main gerado!)
base_node.c
camera_node.c
conveyor_node.c
piston_node.c
safety_node.c
syscalls.c
usart.c
gpio.c
```

### 🔨 Passo 5: Compilar

1. No STM32CubeIDE: **Project > Build Project** (ou Ctrl+B)
2. Aguarde a compilação
3. Deve aparecer: **Build Finished. 0 errors, X warnings**

### 📥 Passo 6: Descarregar no STM32

1. Conecte o **Nucleo board** via USB
2. **Run > Debug** (ou F11)
3. Clique **OK** se perguntar sobre perspectiva Debug
4. Aguarde o flash
5. Clique em **Resume** (F8) para executar

### 📊 Passo 7: Monitorar UART

#### Windows:
1. Abra **PuTTY** ou **TeraTerm**
2. Vá em **Device Manager** para ver a porta COM do ST-Link
3. Configure:
   - **Serial line**: `COM3` (ou a porta correta)
   - **Speed**: `115200`
4. Conecte!

#### Linux/Mac:
```bash
# Descubra a porta
ls /dev/tty*

# Conecte com screen
screen /dev/ttyACM0 115200

# Ou com minicom
minicom -D /dev/ttyACM0 -b 115200
```

### ✅ Saída Esperada

```
╔════════════════════════════════════════════════════════════╗
║   SISTEMA DE ESTEIRA INDUSTRIAL - STM32L053R8             ║
║   Sistema Distribuído de Tempo Real com FreeRTOS          ║
╚════════════════════════════════════════════════════════════╝

[INIT] Inicializando nós do sistema...
[MAIN] Inicializando Nó de Segurança (PRIORIDADE MÁXIMA)...
...
[SIMULACAO] ===== Simulando detecção de VERDE =====
```

### 🎯 Teste de Funcionamento

1. **LED LD2 (PA5)** deve piscar a cada 500ms ✓
2. **UART** deve mostrar mensagens ✓
3. **USER Button (PC13)** ao pressionar deve ativar emergência ✓

---

## 🐛 Problemas Comuns

### ❌ Erro: "region RAM overflowed"

**Causa**: Memória RAM insuficiente (STM32L053R8 tem apenas 8KB)

**Solução**: Edite `Core/Inc/FreeRTOSConfig.h`:
```c
#define configTOTAL_HEAP_SIZE    ((size_t)3072)  // Reduza de 4096 para 3KB
```

---

### ❌ Printf não aparece no terminal

**Verificações**:
1. ✓ USART2 configurado em 115200 baud?
2. ✓ `syscalls.c` está no projeto?
3. ✓ Terminal serial na porta correta?

**Teste direto**:
```c
// No main, após HAL_Init():
char msg[] = "TESTE\r\n";
HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
```

---

### ❌ Sistema trava ou reseta

**Causa**: Stack overflow

**Solução**: Edite `Core/Inc/stm32_config.h`:
```c
#define SAFETY_TASK_STACK_SIZE      64   // Reduza de 128
#define CAMERA_TASK_STACK_SIZE      128  // Reduza de 256
```

---

### ❌ FreeRTOS não compila

**Verificação**:
1. FreeRTOS foi habilitado no CubeMX?
2. Código foi gerado novamente?
3. Include paths corretos?

**Solução**: Delete o projeto e refaça do Passo 1.

---

## 📚 Próximos Passos

Depois que estiver funcionando:

1. 📖 Leia o [README_STM32.md](README_STM32.md) completo
2. 🎨 Veja os [DIAGRAMAS.md](DIAGRAMAS.md)
3. ⏱️ Entenda os [CALCULO_TEMPO.md](CALCULO_TEMPO.md)
4. 🔧 Customize o hardware conforme sua aplicação

---

## 📞 Precisa de Ajuda?

- **Documentação**: [README_STM32.md](README_STM32.md)
- **Setup Detalhado**: [STM32_SETUP_GUIDE.md](STM32_SETUP_GUIDE.md)
- **ST Community**: https://community.st.com/
- **FreeRTOS Forum**: https://forums.freertos.org/

---

**Tempo estimado**: 15-20 minutos (primeira vez)

**Boa sorte! 🚀**
