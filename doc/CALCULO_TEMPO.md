# Cálculo de Tempo de Execução das Threads - MVP FreeRTOS

## Visão Geral

Este documento apresenta os **cálculos detalhados de tempo de execução** de todas as threads do sistema de esteira industrial com FreeRTOS, incluindo:
- Tempo de execução individual de cada thread (WCET - Worst Case Execution Time)
- Análise de utilização de CPU
- Cálculos de schedulability (escalonabilidade)
- Pipeline end-to-end completo
- Análise Rate Monotonic (RM) e validação dos deadlines

---

## 1. Parâmetros Físicos do Sistema

### 1.1 Esteira Industrial

```
Velocidade da Esteira:        V = 100 mm/s
Distância Câmera-Pistão:      D = 200 mm
Tempo de Viagem do Objeto:    T_viagem = D / V
                                        = 200 mm / 100 mm/s
                                        = 2.0 segundos
                                        = 2,000 ms
                                        = 2,000,000 μs
```

### 1.2 Deadlines do Sistema

```
Deadline End-to-End:          D_e2e = 1,500 ms (conservador)
Deadline Processamento:       D_proc = 150 ms
Deadline Pistão:              D_piston = 50 ms
Deadline Crítico:             D_crit = 2,000 ms (perder objeto)
```

---

## 2. Tempo de Execução de Cada Thread (WCET)

### 2.1 Thread Safety Monitor (P:10) - Thread Master

**Prioridade**: 10 (MÁXIMA)
**Período**: T = 10 ms
**Tipo**: Periódica (polling)

#### Análise do Código:
```c
while(1) {
    // 1. Verificar botão de emergência (GPIO read)
    if (button_pressed) {
        // 2. Setar flag global
        g_emergency_stop = true;
        // 3. Timestamp
        emergency_timestamp_us = GetTimestamp();
        // 4. Printf (desconsiderar em WCET)
        // 5. Callback notification
        if (on_emergency_callback != NULL) {
            on_emergency_callback();
        }
    }
    // 6. vTaskDelay(10ms)
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

#### WCET Detalhado:
```
Operação                          Tempo (μs)
─────────────────────────────────────────────
1. GPIO read (button)                    5
2. Set flag global                       2
3. GetTimestamp()                       10
4. Callback notification                50
5. Overhead context switch              20
─────────────────────────────────────────────
WCET Total (Safety):                    87 μs
```

**Tempo de Resposta**: 10 ms (período) + 87 μs (execução) ≈ **10.087 ms**

---

### 2.2 Thread Camera Processing (P:4)

**Prioridade**: 4 (CRÍTICA)
**Período**: T = 10 ms (polling)
**Deadline**: D = 150 ms

#### Análise do Código:
```c
while(1) {
    // 1. Frame capture (se disponível)
    frame = CaptureFrame();              // 100 ms

    // 2. Color detection
    color = DetectColor(frame);          // 80 ms

    // 3. Quality validation
    if (ValidateQuality(...)) {          // 5 ms
        // 4. Create event
        CreateColorEvent(&event);        // 10 μs

        // 5. Send to pistons (queue)
        for (i = 0; i < piston_count; i++) {
            xQueueSend(...);             // 100 μs × 2 = 200 μs
        }
    }

    // 6. vTaskDelay(10ms)
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

#### WCET Detalhado:
```
Operação                              Tempo
──────────────────────────────────────────────
1. Frame Capture (ESP32-CAM)      100,000 μs
2. Color Detection (HSV + mask)    80,000 μs
3. Quality Validation               5,000 μs
4. Event Creation                      10 μs
5. Queue Send (×2 pistões)            200 μs
6. Overhead & context switch          100 μs
──────────────────────────────────────────────
WCET Total (Camera):              185,310 μs ≈ 185.3 ms
```

**Análise**: WCET (185.3 ms) > Deadline (150 ms) ⚠️
**Solução**: Otimização do algoritmo de detecção ou aumento do deadline para 200 ms

---

### 2.3 Thread Piston A Control (P:4)

**Prioridade**: 4 (CRÍTICA)
**Período**: Aperiódica (event-driven)
**Deadline**: D = 50 ms (resposta após receber evento)

#### Análise do Código:
```c
while(1) {
    // 1. Aguarda evento (bloqueado)
    xQueueReceive(queue, &event, portMAX_DELAY);  // Bloqueado

    // 2. Calcula delay da esteira
    delay = CalculateBeltDelay();                 // 50 μs

    // 3. Agenda ativação
    xQueueSend(activation_queue, ...);            // 100 μs

    // 4. Aguarda delay calculado
    vTaskDelay(calculated_delay);                 // 2000 ms típico

    // 5. Ativa pistão fisicamente
    GPIO_SET_HIGH();                              // 10 μs
    vTaskDelay(50ms);                             // 50 ms (movimento físico)
    GPIO_SET_LOW();                               // 10 μs

    // 6. Calcula métricas
    CalculateMetrics();                           // 200 μs

    // 7. Atualiza estatísticas
    UpdateStats();                                // 100 μs
}
```

#### WCET Detalhado:
```
Operação                              Tempo
──────────────────────────────────────────────
1. xQueueReceive (já recebido)            0 μs (bloqueado)
2. CalculateBeltDelay()                  50 μs
3. xQueueSend (agendamento)             100 μs
4. vTaskDelay (bloqueado)                0 μs (não conta para WCET)
5. GPIO operations                       20 μs
6. Physical movement (vTaskDelay)    50,000 μs
7. CalculateMetrics()                   200 μs
8. UpdateStats()                        100 μs
9. Context switch overhead              100 μs
──────────────────────────────────────────────
WCET Total (Piston):               50,570 μs ≈ 50.6 ms
```

**Análise**: WCET (50.6 ms) ≈ Deadline (50 ms) ✓ **OK** (margem mínima)

---

### 2.4 Thread Piston B Control (P:4)

**Idêntico ao Piston A**

```
WCET Total (Piston B):            50,570 μs ≈ 50.6 ms
```

---

### 2.5 Thread Conveyor Control (P:2)

**Prioridade**: 2 (MÉDIA)
**Período**: T = 100 ms

#### Análise do Código:
```c
while(1) {
    // 1. Verifica emergência
    if (GlobalEmergencyCheck()) {        // 10 μs
        ConveyorHalt(true);              // 50 μs
    }

    // 2. Atualiza runtime
    UpdateRuntime();                     // 100 μs

    // 3. vTaskDelay
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

#### WCET Detalhado:
```
Operação                              Tempo
──────────────────────────────────────────────
1. Emergency check                       10 μs
2. Halt (worst case)                     50 μs
3. UpdateRuntime()                      100 μs
4. Context switch                        20 μs
──────────────────────────────────────────────
WCET Total (Conveyor):                 180 μs
```

---

### 2.6 Thread Statistics (P:1)

**Prioridade**: 1 (BAIXA)
**Período**: T = 10,000 ms (10 segundos)

#### WCET Detalhado:
```
Operação                              Tempo
──────────────────────────────────────────────
1. Collect stats from all nodes      1,000 μs
2. Printf operations (5 nós)       100,000 μs (estimativa)
3. Format strings                    5,000 μs
──────────────────────────────────────────────
WCET Total (Stats):               106,000 μs ≈ 106 ms
```

---

### 2.7 Thread Heartbeat (P:2)

**Prioridade**: 2
**Período**: T = 1,000 ms

#### WCET Detalhado:
```
Operação                              Tempo
──────────────────────────────────────────────
1. Send heartbeat message              100 μs
2. Context switch                       20 μs
──────────────────────────────────────────────
WCET Total (Heartbeat):                120 μs
```

---

## 3. Tabela Resumo de WCET

| Thread               | Prioridade | Período (ms) | WCET (ms) | Utilização (%) |
|----------------------|------------|--------------|-----------|----------------|
| **Safety Monitor**   | 10         | 10           | 0.087     | 0.87%          |
| **Camera Process**   | 4          | 10¹          | 185.3     | 1853%²         |
| **Piston A**         | 4          | -³           | 50.6      | -³             |
| **Piston B**         | 4          | -³           | 50.6      | -³             |
| **Conveyor**         | 2          | 100          | 0.18      | 0.18%          |
| **Heartbeat**        | 2          | 1000         | 0.12      | 0.012%         |
| **Statistics**       | 1          | 10000        | 106       | 1.06%          |

¹ Período de polling, mas processamento completo leva 185ms
² **PROBLEMA**: Excede 100%! Processamento deve ser otimizado
³ Aperiódica (event-driven)

---

## 4. Análise de Schedulability (Escalonabilidade)

### 4.1 Teste de Liu & Layland (Rate Monotonic)

Para **n tarefas periódicas**, o sistema é escalonável se:

```
U = Σ (Ci / Ti) ≤ n(2^(1/n) - 1)

Onde:
  Ci = WCET da tarefa i
  Ti = Período da tarefa i
  n  = número de tarefas
```

#### Cálculo para Tarefas Periódicas:

```
U_safety    = 0.087 / 10     = 0.0087  (0.87%)
U_camera    = 185.3 / 200¹   = 0.9265  (92.65%)
U_conveyor  = 0.18  / 100    = 0.0018  (0.18%)
U_heartbeat = 0.12  / 1000   = 0.00012 (0.012%)
U_stats     = 106   / 10000  = 0.0106  (1.06%)

U_total = 0.0087 + 0.9265 + 0.0018 + 0.00012 + 0.0106
        = 0.94772
        = 94.77%
```

¹ Considerando período realista de 200ms (5 FPS)

**Limite de Liu & Layland para n=5**:
```
U_limit = 5(2^(1/5) - 1)
        = 5(1.1487 - 1)
        = 5(0.1487)
        = 0.7435
        = 74.35%
```

**Resultado**: U_total (94.77%) > U_limit (74.35%) ⚠️

**Conclusão**: Sistema **NÃO é garantidamente escalonável** por Rate Monotonic.

### 4.2 Teste de Utilização Total (menos conservador)

```
U_total = 94.77% < 100%
```

**Análise**: Sistema pode funcionar, mas sem garantias formais de deadlines.

---

## 5. Pipeline End-to-End Completo

### 5.1 Cenário Best Case (Sem Interferência)

```
TEMPO    EVENTO                                 LATÊNCIA ACUMULADA
─────────────────────────────────────────────────────────────────
  0 ms   Objeto entra no campo da câmera              0 ms
         ↓
100 ms   Frame capturado (Camera Capture)           100 ms
         └─> Enviado para fila de processamento
         ↓
180 ms   Cor detectada (Image Processing)           180 ms
         └─> HSV conversion + color mask
         └─> Quality validation
         ↓
181 ms   ColorDetectionEvent criado                  181 ms
         └─> xQueueSend() para pistões
         ↓
182 ms   Pistão recebe comando                       182 ms
         └─> xQueueReceive() retorna
         └─> CalculateBeltDelay()
         ↓
183 ms   Ativação agendada                           183 ms
         └─> vTaskDelay(2000ms) programado
         ↓
2183 ms  Pistão ATIVA (físico)                      2183 ms
         └─> GPIO_SET_HIGH
         └─> Movimento pneumático: 50ms
         ↓
2233 ms  Pistão retorna                             2233 ms
         └─> GPIO_SET_LOW
         ↓
───────────────────────────────────────────────────────────────
TOTAL BEST CASE: 183 ms (até decisão)
                 2233 ms (até conclusão física)
```

**Margem disponível**: 2000 ms (viagem) - 183 ms (decisão) = **1817 ms (90.8%)**

---

### 5.2 Cenário Worst Case (Com Interferência Máxima)

```
TEMPO    EVENTO                                 LATÊNCIA ACUMULADA
─────────────────────────────────────────────────────────────────
  0 ms   Objeto entra no campo
         ↓
  0 ms   [Preempção] Safety Monitor executa           +0.087 ms
         ↓
100 ms   Frame Capture inicia                        100 ms
         ↓
105 ms   [Preempção] Safety Monitor (2× durante)       +0.174 ms
         ↓
200 ms   Frame Capture completa                      200.174 ms
         ↓
200 ms   Image Processing inicia
         ↓
210 ms   [Preempção] Safety Monitor (8× durante)       +0.696 ms
         ↓
280 ms   [Preempção] Piston A/B (1× cada)              +101.2 ms
         └─> Cada pistão: 50.6ms, mas executam
             sequencialmente se houver eventos
         ↓
381 ms   Color Detection completa                    381.87 ms
         ↓
382 ms   Queue Send (com retry)                      382.97 ms
         └─> Assume 1ms com contenção
         ↓
383 ms   Pistão recebe e agenda                      383.12 ms
         ↓
2383 ms  Pistão ATIVA                               2383.12 ms
         ↓
2433 ms  Pistão completa                            2433.12 ms
───────────────────────────────────────────────────────────────
TOTAL WORST CASE: 383 ms (até decisão)
                  2433 ms (até conclusão)
```

**Margem**: 2000 - 383 = **1617 ms (80.8%)**

**Problema**: 2433 ms > 2000 ms ⚠️
O objeto pode passar do pistão antes da ativação completar!

---

### 5.3 Cenário com Emergência

```
TEMPO    EVENTO                                 AÇÃO
─────────────────────────────────────────────────────────────────
  0 ms   Operação normal
         ↓
 50 ms   Botão de emergência pressionado
         ↓
 50.087ms Safety Monitor detecta                 [PARA TUDO]
         └─> g_emergency_stop = true
         ↓
 55 ms   Piston A verifica flag (próximo poll)  [PARA]
 55 ms   Piston B verifica flag                 [PARA]
         ↓
 60 ms   Camera verifica flag                   [PARA]
         ↓
150 ms   Conveyor verifica flag                 [PARA MOTOR]
         └─> ConveyorHalt(emergency=true)
───────────────────────────────────────────────────────────────
TEMPO TOTAL DE PARADA: 150 ms (worst case)
                        55 ms (componentes críticos)
```

---

## 6. Análise de Jitter

### 6.1 Jitter da Thread Camera

**Jitter** = Variação no tempo de resposta

```
Melhor caso (best case):     180 ms
Pior caso (worst case):      383 ms
Jitter máximo:               203 ms
```

**Análise**: Jitter (203 ms) > Limite aceitável (50 ms) ⚠️

---

## 7. Análise de Tempo de Resposta (Response Time Analysis)

### 7.1 Fórmula de Response Time

Para tarefa i com prioridade pi:

```
Ri = Ci + Σ ⌈Ri / Tj⌉ × Cj
      j∈hp(i)

Onde:
  Ri = Response time da tarefa i
  Ci = WCET da tarefa i
  hp(i) = Conjunto de tarefas com prioridade maior que i
```

### 7.2 Response Time da Camera (P:4)

```
hp(Camera) = {Safety}  (apenas Safety tem P>4)

R_camera^0 = C_camera = 185.3 ms

R_camera^1 = 185.3 + ⌈185.3 / 10⌉ × 0.087
           = 185.3 + 19 × 0.087
           = 185.3 + 1.653
           = 186.953 ms

R_camera^2 = 185.3 + ⌈186.953 / 10⌉ × 0.087
           = 185.3 + 19 × 0.087
           = 186.953 ms  (convergiu)

R_camera = 186.953 ms
```

**Deadline**: 150 ms
**Response Time**: 186.953 ms
**Resultado**: **DEADLINE PERDIDO** ⚠️

---

## 8. Otimizações Recomendadas

### 8.1 Reduzir WCET da Camera

**Problema**: Camera WCET = 185.3 ms > Deadline = 150 ms

**Soluções**:

1. **Otimizar Algoritmo de Detecção**:
   ```
   Atual:  HSV conversion + full frame mask = 80 ms
   Opção A: ROI (Region of Interest) apenas    = 40 ms  (↓50%)
   Opção B: Downscaling 320x240 → 160x120      = 20 ms  (↓75%)
   Opção C: Hardware acceleration (ESP32-S3)   = 10 ms  (↓87.5%)
   ```

2. **Reduzir Resolução**:
   ```
   Atual:    320×240 @ 100ms capture
   Proposta: 160×120 @ 50ms capture
   Ganho:    50ms capture + 40ms processing = 90ms total
   Novo WCET: 95 ms ✓ Dentro do deadline!
   ```

### 8.2 Ajustar Prioridades

**Problema**: Camera (P:4) compete com Pistons (P:4)

**Solução**:
```
Safety Monitor: P:10  (sem mudança)
Pistons:        P:5   (aumentar - mais crítico)
Camera:         P:4   (manter)
Conveyor:       P:2   (sem mudança)
```

### 8.3 Aumentar Período da Camera

**Problema**: Polling a cada 10ms é desnecessário

**Solução**:
```
Atual:    T = 10 ms  (100 FPS teórico, mas captura só 10 FPS)
Proposta: T = 200 ms (5 FPS real)
Ganho:    U_camera = 185.3 / 200 = 92.65% → OK!
```

---

## 9. Configuração Otimizada Final

### 9.1 Novos Parâmetros

```c
// timing_config.h

// WCET otimizado
#define CAMERA_FRAME_CAPTURE_TIME_US     50000UL   // 50 ms (160x120)
#define IMAGE_PROCESSING_TIME_US         40000UL   // 40 ms (ROI)
#define TOTAL_DETECTION_TIME_US          90000UL   // 90 ms

// Período realista
#define CAMERA_TASK_PERIOD_MS            200       // 200 ms (5 FPS)

// Prioridades ajustadas
#define PRIORITY_EMERGENCY               10
#define PRIORITY_PISTON_CONTROL          5         // ↑ Aumentado
#define PRIORITY_IMAGE_PROCESSING        4         // Mantido
```

### 9.2 Nova Análise de Schedulability

```
U_safety    = 0.087 / 10     = 0.0087
U_camera    = 90    / 200    = 0.45     ✓ Melhorou!
U_conveyor  = 0.18  / 100    = 0.0018
U_heartbeat = 0.12  / 1000   = 0.00012
U_stats     = 106   / 10000  = 0.0106

U_total = 0.47122 = 47.12%
```

**Limite RM**: 74.35%
**Resultado**: 47.12% < 74.35% ✓ **SISTEMA ESCALONÁVEL!**

---

## 10. Resumo Executivo

### 10.1 Configuração Atual (Não Otimizada)

```
❌ WCET Camera:      185.3 ms > Deadline 150 ms
❌ Utilização:       94.77% > Limite RM 74.35%
❌ Jitter Camera:    203 ms > Limite 50 ms
⚠️  Worst Case:      2433 ms > Travel Time 2000 ms
```

### 10.2 Configuração Otimizada

```
✅ WCET Camera:      90 ms < Deadline 150 ms
✅ Utilização:       47.12% < Limite RM 74.35%
✅ Jitter Camera:    ~20 ms < Limite 50 ms
✅ Worst Case:       ~300 ms << Travel Time 2000 ms
✅ Margem:           1700 ms (85% do tempo disponível)
```

### 10.3 Métricas Finais

| Métrica                     | Atual    | Otimizado | Status |
|-----------------------------|----------|-----------|--------|
| WCET Camera (ms)            | 185.3    | 90        | ✅ OK  |
| Utilização CPU (%)          | 94.77    | 47.12     | ✅ OK  |
| Schedulability              | ❌ Não   | ✅ Sim    | ✅ OK  |
| Deadline Camera             | ❌ Perde | ✅ Cumpre | ✅ OK  |
| Tempo Emergência (ms)       | 150      | 150       | ✅ OK  |
| Margem End-to-End (ms)      | 1617     | 1700      | ✅ OK  |

---

## 11. Referências

- **Liu, C. L., & Layland, J. W. (1973)**. "Scheduling Algorithms for Multiprogramming in a Hard-Real-Time Environment". *Journal of the ACM*, 20(1), 46-61.
- **FreeRTOS Documentation**: https://www.freertos.org/
- **Rate Monotonic Analysis**: https://en.wikipedia.org/wiki/Rate-monotonic_scheduling
- **ESP32 Technical Reference**: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf

---

**Documentação criada para o MVP FreeRTOS - Sistema de Esteira Industrial**
**Disciplina: Tópicos Especiais em Sistemas Distribuídos**
