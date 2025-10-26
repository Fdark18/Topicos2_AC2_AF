# 📁 Estrutura do Projeto - STM32L053R8 + FreeRTOS

## ✅ Estrutura Limpa e Organizada

```
mvp_freertos/
│
├── 📁 Core/                           ← CÓDIGO FONTE PRINCIPAL
│   ├── 📁 Inc/                        ← Headers (10 arquivos)
│   │   ├── stm32_config.h            ⭐ Config hardware STM32L053R8
│   │   ├── FreeRTOSConfig.h          ⭐ Config FreeRTOS (4KB heap)
│   │   ├── timing_config.h           ⭐ Configurações de timing real
│   │   ├── base_node.h
│   │   ├── camera_node.h
│   │   ├── conveyor_node.h
│   │   ├── piston_node.h
│   │   ├── safety_node.h
│   │   ├── usart.h
│   │   └── gpio.h
│   │
│   └── 📁 Src/                        ← Source files (9 arquivos)
│       ├── main_stm32.c              ⭐ Main para STM32 HAL
│       ├── base_node.c
│       ├── camera_node.c
│       ├── conveyor_node.c
│       ├── piston_node.c
│       ├── safety_node.c
│       ├── syscalls.c                ⭐ Printf via UART
│       ├── usart.c
│       └── gpio.c
│
├── 📁 Drivers/                        ← (Vazio - gerado pelo CubeMX)
│   ├── CMSIS/
│   └── STM32L0xx_HAL_Driver/
│
├── 📁 Middlewares/                    ← (Vazio - gerado pelo CubeMX)
│   └── Third_Party/
│       └── FreeRTOS/
│
├── 📄 README.md                       ⭐ COMECE AQUI! (Guia completo)
├── 📄 QUICK_START_STM32.md            ⭐ Guia rápido 5 minutos
├── 📄 CALCULO_TEMPO.md                📊 Análise de timing
├── 📄 DIAGRAMAS.md                    📊 Diagramas de execução
├── 📄 README_ORIGINAL.md              📚 Documentação original (referência)
├── 📄 .gitignore
└── 📄 ESTRUTURA_PROJETO.md            📋 Este arquivo
```

## 🎯 Arquivos Essenciais para Compilação

### Para o STM32CubeIDE usar:

#### 1️⃣ **Headers (Core/Inc/)**
- `stm32_config.h` - Configuração de pinos e hardware
- `FreeRTOSConfig.h` - Configuração do FreeRTOS
- `timing_config.h` - Parâmetros de tempo real
- `*.h` - Headers dos nós do sistema

#### 2️⃣ **Source (Core/Src/)**
- `main_stm32.c` - Ponto de entrada
- `syscalls.c` - Suporte para printf
- `*.c` - Implementação dos nós

#### 3️⃣ **Gerados pelo CubeMX (Drivers/ e Middlewares/)**
- HAL drivers
- FreeRTOS source code
- CMSIS

## 📊 Estatísticas do Projeto

### Arquivos de Código
- **Headers**: 10 arquivos
- **Sources**: 9 arquivos
- **Total**: 19 arquivos de código

### Documentação
- **Guias**: 2 arquivos (README.md, QUICK_START.md)
- **Técnica**: 2 arquivos (CALCULO_TEMPO.md, DIAGRAMAS.md)
- **Referência**: 1 arquivo (README_ORIGINAL.md)

### Memória Estimada
- **Código fonte**: ~30-40 KB
- **FreeRTOS**: ~10-15 KB
- **HAL drivers**: ~10-15 KB
- **Total Flash**: ~50-60 KB (de 64 KB disponíveis) ✅
- **RAM utilizada**: ~6-7 KB (de 8 KB disponíveis) ✅

## 🗑️ Arquivos Removidos (Limpeza)

### Duplicados (estavam na raiz):
- ❌ base_node.c/h
- ❌ camera_node.c/h
- ❌ conveyor_node.c/h
- ❌ piston_node.c/h
- ❌ safety_node.c/h
- ❌ gpio.c/h
- ❌ usart.c/h
- ❌ syscalls.c
- ❌ timing_config.h
- ❌ stm32_config.h
- ❌ main.c (ESP32 version)

### Documentação LaTeX/ABNT:
- ❌ DOCUMENTACAO_TECNICA_ABNT.md
- ❌ README_LATEX.md
- ❌ trabalho_academico.tex
- ❌ references.bib
- ❌ STM32_SETUP_GUIDE.md

### Outros:
- ❌ partes/ (pasta vazia)
- ❌ REORGANIZACAO_COMPLETA.md (temporário)

## 🚀 Como Usar Esta Estrutura

### Passo 1: Abrir no STM32CubeIDE
```
File > New > STM32 Project
Board Selector: NUCLEO-L053R8
```

### Passo 2: Copiar Arquivos
```bash
# Copiar headers
cp Core/Inc/* [seu_projeto]/Core/Inc/

# Copiar sources
cp Core/Src/* [seu_projeto]/Core/Src/
```

### Passo 3: Configurar CubeMX
Ver [QUICK_START_STM32.md](QUICK_START_STM32.md) para detalhes.

### Passo 4: Compilar
```
Project > Build Project
```

### Passo 5: Descarregar
```
Run > Debug (F11)
```

## 📚 Ordem de Leitura Recomendada

1. **[README.md](README.md)** ← Comece aqui! ⭐
2. **[QUICK_START_STM32.md](QUICK_START_STM32.md)** ← Guia prático
3. **[Core/Inc/stm32_config.h](Core/Inc/stm32_config.h)** ← Config hardware
4. **[Core/Inc/FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h)** ← Config FreeRTOS
5. **[Core/Src/main_stm32.c](Core/Src/main_stm32.c)** ← Main
6. **[DIAGRAMAS.md](DIAGRAMAS.md)** ← Visualização
7. **[CALCULO_TEMPO.md](CALCULO_TEMPO.md)** ← Análise tempo real

## 🎯 Mapeamento Hardware (STM32L053R8)

```
┌─────────────────────────────────┐
│  STM32L053R8 Nucleo Pinout     │
├─────────────────────────────────┤
│ PC13 → USER Button (Emergency) │
│ PA2  → USART2_TX (Debug)       │
│ PA3  → USART2_RX (Debug)       │
│ PA5  → LED LD2 (Status)        │
│ PA8  → Piston A Output         │
│ PA9  → Piston B Output         │
│ PA0  → TIM2_CH1 (PWM Motor)    │
│ PB0  → LED Error (External)    │
└─────────────────────────────────┘
```

## ⚙️ Especificações STM32L053R8

| Característica | Valor |
|----------------|-------|
| **Core** | ARM Cortex-M0+ |
| **Clock** | 32 MHz |
| **Flash** | 64 KB |
| **RAM** | 8 KB |
| **GPIO** | 37 pinos |
| **USART** | 2x |
| **Timers** | TIM2, TIM21, TIM22 |
| **Package** | LQFP64 |

## 🔧 Configurações Importantes

### FreeRTOS (FreeRTOSConfig.h)
```c
configTOTAL_HEAP_SIZE       = 4096     // 4 KB (50% RAM)
configMAX_PRIORITIES        = 6        // 0-5
configTICK_RATE_HZ          = 1000     // 1 ms tick
configMINIMAL_STACK_SIZE    = 64       // 256 bytes
```

### Prioridades das Tasks
```c
PRIORITY_SAFETY      = 5   // MÁXIMA
PRIORITY_PISTON      = 4   // Crítica
PRIORITY_CAMERA      = 4   // Crítica
PRIORITY_CONVEYOR    = 2   // Média
PRIORITY_MONITOR     = 1   // Baixa
```

### Stack das Tasks
```c
SAFETY_TASK_STACK    = 128 words (512 B)
CAMERA_TASK_STACK    = 256 words (1 KB)
PISTON_TASK_STACK    = 128 words (512 B)
CONVEYOR_TASK_STACK  = 128 words (512 B)
MONITOR_TASK_STACK   = 256 words (1 KB)
```

## ✅ Checklist de Validação

Antes de compilar, verifique:

- [x] Estrutura Core/Inc e Core/Src existe
- [x] 10 headers em Core/Inc/
- [x] 9 sources em Core/Src/
- [x] FreeRTOSConfig.h configurado
- [x] stm32_config.h para STM32L053R8
- [x] main_stm32.c usando HAL
- [x] syscalls.c para printf
- [x] Arquivos duplicados removidos
- [x] Documentação organizada

## 🐛 Troubleshooting

### Erro: "region RAM overflowed"
**Solução**: Reduza `configTOTAL_HEAP_SIZE` em FreeRTOSConfig.h

### Erro: "undefined reference to printf"
**Solução**: Adicione syscalls.c ao projeto

### Sistema trava
**Solução**: Reduza stack das tasks em stm32_config.h

## 📞 Suporte

- **Documentação completa**: [README.md](README.md)
- **Guia rápido**: [QUICK_START_STM32.md](QUICK_START_STM32.md)
- **ST Community**: https://community.st.com/
- **FreeRTOS Forum**: https://forums.freertos.org/

---

**Projeto limpo e pronto para compilação! ✅**

**Última atualização**: 2025-10-25
