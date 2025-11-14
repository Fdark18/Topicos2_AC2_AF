# Simulador POSIX do Projeto

Este repositório contém um ambiente de simulação baseado em POSIX localizado em `validacao/`. Ele recompila o mesmo código de lógica dos nós (câmera, pistões, esteira e segurança) e executa as tarefas FreeRTOS por meio de um micro-kernel implementado com threads POSIX (ou Win32, no Windows).
O objetivo principal é permitir que todo o sistema distribuído seja testado diretamente no computador, sem depender do Nucleo STM32 ou do hardware físico.

---

## Benefícios

1. **Iteração rápida**
   É possível depurar a orquestração de tarefas, filas, deadlines e callbacks em poucos segundos, sem necessidade de realizar flash no microcontrolador.

2. **Observabilidade ampliada**
   O console exibe logs detalhados sobre latência, deadlines, QoS, ativação dos pistões e eventos de emergência, facilitando diagnósticos que seriam limitados apenas a LEDs e UART no hardware real.

3. **Portabilidade para documentação e demonstrações**
   O binário `esteira_sim` serve como referência teórica do comportamento de tempo real, útil para apresentações acadêmicas, relatórios e revisões de código.

---

## Malefícios e Limitações

1. **Não substitui o hardware real**
   Tempos de GPIO, interrupções e efeitos físicos não são simulados. A validação é lógica, mas não garante compatibilidade elétrica ou precisão de desempenho final.

2. **Temporização dependente do host**
   O determinismo está limitado à precisão das funções de temporização (`sleep/usleep`) do sistema operacional, podendo variar com a carga da máquina.

3. **Ausência de periféricos reais**
   Os drivers HAL são implementados como stubs. Dessa forma, falhas específicas de periféricos (ADC, TIM, DMA etc.) só podem ser identificadas no hardware STM32.

---

## Resumo do Projeto

O projeto completo implementa uma linha de esteira industrial com FreeRTOS. Os módulos funcionam da seguinte forma:

* **Câmera:** detecta a cor dos objetos.
* **Safety Node:** monitora condições de emergência.
* **Conveyor Node:** controla a velocidade da esteira.
* **Pistons Node:** opera dois pistões responsáveis pela separação de objetos (azul → esquerda, verde → direita).

O FreeRTOS gerencia tarefas independentes como: Heartbeat, Monitoramento, Processamento de Imagem, Controle da Esteira, Controle dos Pistões, Estatísticas e Simulação.

O simulador POSIX reproduz esse ambiente, permitindo validar diversos cenários, como:

1. **Triagem automática por cor**
   Confirmar que objetos verdes acionam o pistão direito e objetos azuis acionam o pistão esquerdo dentro dos tempos definidos.

2. **Estado de emergência por cor inesperada**
   Simular detecção de objetos na cor vermelha, ativar o estado de parada e validar que a esteira somente reinicia após o reset correto.

3. **Demonstração de métricas de tempo real**
   Avaliar deadlines, QoS e jitter para fins acadêmicos, demonstrando o comportamento soft real-time do sistema mesmo sem o hardware STM32.

