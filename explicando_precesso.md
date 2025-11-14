1- Projeto: controla uma esteira industrial inteligente — a câmera identifica a cor das peças, o RTOS (FreeRTOS/portado também para POSIX no simulador) roteia eventos e aciona pistões dedicados, enquanto o nó de segurança monitora emergências. Simulador POSIX permite ensaiar toda a lógica com threads/pthreads antes de ir ao STM32. Possíveis usos: (1) linha de triagem de embalagens por cor (verde para uma saída, azul para outra); (2) célula didática de RTOS para demonstrar deadlines, QoS e paradas de emergência em cursos de sistemas embarcados.

2-
A câmera roda como uma tarefa FreeRTOS dedicada e, ao detectar uma cor, coloca um Message_t em sua fila TX; um roteador (tarefa cooperativa) lê esse evento no mesmo instante e o replica para as filas RX dos pistões. Isso garante que a informação saia da câmera, chegue ao pistão correto e gere uma ativação sem polling, apenas via filas/ISR-safe.
Cada pistão executa sua própria tarefa de controle (PistonNode_ControlTask), bloqueada na activation_queue. Assim que o evento chega, ela calcula o atraso necessário (tempo que o objeto leva para sair da câmera e chegar ao atuador) com base na velocidade da esteira e na distância configurada, depois agenda o movimento físico — tudo dentro de deadlines monitorados.
A velocidade da esteira é controlada pelo ConveyorNode, que também roda em tempo real: quando você ajusta a velocidade, o node atualiza belt_speed_mm_s e isso é propagado aos pistões (a função BaseNode_CalculateBeltDelay usa esse valor para recalcular o tempo de viagem). O resultado é um laço fechado: câmera detecta → mensagem chega ao pistão → pistão espera exatamente o tempo de transporte calculado com a velocidade atual → pistão aciona, mantendo o determinismo e o alinhamento com o movimento da esteira.

3 -
Imagine três pessoas trabalhando lado a lado:

Câmera: é quem observa a esteira. Quando vê uma peça azul ou verde, ela imediatamente avisa os colegas (“Tenho um azul chegando!”). Isso acontece na mesma hora, sem esperar nenhum turno.
Pistões: são os braços que empurram as peças. Cada braço ouve apenas o aviso que lhe interessa (o da cor que precisa separar). Assim que recebe a mensagem, calcula quanto tempo falta para a peça chegar e já se prepara para empurrá-la exatamente no momento certo.
Controle da esteira: é o operador do motor. Se alguém muda a velocidade, essa informação é contada para todo mundo. A câmera passa a medir o tempo considerando a nova velocidade e os pistões reajustam o momento de empurrar.
Como todos conversam em tempo real (sem filas ou recados atrasados), a câmera não perde peças, os pistões batem sempre na hora certa e a esteira vai ajustando o ritmo conforme a situação.

4- 
O simulador POSIX é uma versão especial do FreeRTOS criada para rodar em um computador comum (Windows/Linux), usando APIs do sistema operacional (POSIX) para emular o comportamento do kernel FreeRTOS, sem precisar de um microcontrolador como o STM32.

Ele não simula hardware real, mas sim as tarefas, filas, semáforos e o agendador do FreeRTOS.


5-
O simulador POSIX executa o FreeRTOS como um programa normal no computador, usando pthreads e timers POSIX para imitar tarefas e o tick do kernel.

Ele não simula hardware do STM32, mas sim a lógica do sistema operacional embarcado: tasks, filas, semáforos e escalonamento.

É uma forma rápida e segura de desenvolver e testar firmware FreeRTOS antes de portar o código para um microcontrolador real.

6-
                +-----------------------------+
                |      código FreeRTOS     |
                |  (tasks, filas, semáforos)  |
                +---------------+-------------+
                                |
                                v
                +-----------------------------+
                |      Kernel do FreeRTOS     |
                | (mesmo usado no STM32)      |
                +---------------+-------------+
                                |
                                v
        +------------------------------------------------+
        |              Portabilidade POSIX               |
        |                                                |
        |  Task -> pthread --------------------------+   |
        |  Delay -> nanosleep()                      |   |
        |  Tick -> timer POSIX/SIGALRM               |   |
        |  Mutex/Semáforo -> pthread_mutex/condvar   |   |
        +----------------------+----------------------+   |
                               |                          |
                               v                          |
                    +-------------------------+           |
                    |  Sistema Operacional    | <---------+
                    | (Linux/Windows/Mac)     |
                    +---------------+---------+
                                    |
                                    v
                    +-----------------------------+
                    |      Execução no PC         |
                    |  (sem hardware STM32 real)  |
                    +-----------------------------+
