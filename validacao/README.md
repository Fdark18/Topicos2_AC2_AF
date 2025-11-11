# Validação sem Hardware – Sistema de Esteira FreeRTOS

Este diretório contém um *harness* de validação que permite exercitar o código
dos nós FreeRTOS inteiramente no host (Linux/WSL/macOS ou Windows com pthreads),
sem depender do Nucleo STM32. Ele executa exatamente os módulos de lógica do
projeto (`base_node`, `camera_node`, `piston_node`, `conveyor_node`,
`safety_node`) sobre um micro‐kernel POSIX que simula a API do FreeRTOS
(filas, tarefas, semáforos e `TickCount`). Assim é possível verificar a
orquestração de tarefas, comunicação entre nós e tratamento de emergência.

## Estrutura

```
validacao/
├── CMakeLists.txt          # Gera o executável host `esteira_sim`
├── include/                # Stubs da API FreeRTOS (FreeRTOS.h, task.h, …)
│   └── freertos_sim.h
├── src/
│   ├── freertos_sim.c      # Implementação POSIX das filas/tarefas/semáforos
│   └── validation_main.c   # Orquestra nós, tarefas auxiliares e cenários
```

Os fontes originais em `Core/Src/*.c` são compilados junto com o simulador –
qualquer alteração nesses arquivos é automaticamente exercitada aqui.

## Pré-requisitos

- Toolchain C11 com `cmake` ≥ 3.16 e `gcc`/`clang` com suporte a `pthread`.
  - Em Windows, recomenda-se usar WSL2 ou MSYS2/MinGW (com `pthread` instalado).
- Nenhum acesso ao hardware físico é necessário.

## Como compilar

```bash
cd validacao
cmake -S . -B build
cmake --build build
```

O binário `build/esteira_sim` (ou `esteira_sim.exe`) será gerado.

> Sem CMake? Rode `make -C validacao` para usar o `Makefile` simples incluso, que
> compila tudo em um único comando (`gcc`/`clang` precisam oferecer `-pthread`).

## Como executar o cenário padrão

```bash
cd validacao/build
./esteira_sim
```

O runner realiza automaticamente:

1. Inicialização e *start* dos nós de Segurança, Esteira, Câmera e Pistões,
   exatamente como ocorre no `main.c` do firmware.
2. Criação das tarefas auxiliares:
   - `BlinkTask`: emite *heartbeat* textual a cada 1s.
   - `SimulationTask`: alterna detecções de verde/azul a cada 5s,
     aciona `CameraNode_SimulateDetection` e marca a saída do objeto.
   - `StatsTask`: imprime relatórios consolidados a cada 10s.
3. Comando `ConveyorNode_Run` para validar a transição RUN/STOP.
4. Após 15s, simula o acionamento do botão de emergência (seta
   `safety.button_pressed`), verificando se todos os nós entram em modo seguro.
5. Mantém execução contínua (Ctrl+C para encerrar) para que novas alterações
   possam ser observadas em tempo real.

Os *logs* impressos confirmam métricas de tempo real, QoS, ativações dos pistões
e estados da esteira, permitindo comparação direta com o comportamento esperado
em hardware.

## Customizações úteis

- **Novos testes**: Edite `validacao/src/validation_main.c` e adicione novas
  tarefas ou sequências (ex.: bursts de objetos, jitter extremo, reset da
  emergência).
- **Integração com CI**: O binário retorna `0` em caso de inicialização bem
  sucedida — pode ser rodado em pipelines como gate pré-flash.
- **Cobertura/asan**: habilite flags extras via `CMAKE_C_FLAGS` para coletar
  cobertura ou detectar *memory issues*.

## Limitações atuais

- Temporizações dependem do `sleep` do host; para resultados determinísticos,
  execute em máquinas pouco carregadas.
- O simulador não representa periféricos HAL/GPIO; porém, os pontos onde o
  hardware seria acionado permanecem instrumentados por logs.
- As tarefas não são finalizadas automaticamente ao sair; finalize o processo
  ao término do teste (Ctrl+C) ou adapte o loop final para encerrar.

Com esse fluxo é possível validar toda a lógica distribuída antes de se obter o
Nucleo físico, acelerando o desenvolvimento e evitando regressões. Ajuste os
cenários conforme necessário para o seu plano de testes.*** End Patch
