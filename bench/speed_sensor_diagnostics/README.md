# Diagnóstico independente do sensor Hall de velocidade

Projeto de bancada independente para ESP32-WROOM-32 e ESP-IDF 6.0.2. Reúne
oito modos selecionáveis pelo monitor serial (T0–T7) para separar problemas de
boot, entrada GPIO, captura de bordas, geração de pulsos, forma de onda e
processamento. Não usa GPS, SD, LoRa ou sensores do firmware do Baja.

O componente `speed_sensor` foi extraído de `components/speed_sensor` da branch
`feat/speed-sensor`, commit `434ddd1ff0eba950d8a9e82c242244f1edec58cf`. T6 usa
diretamente o driver. Os outros modos usam captura GPIO própria, no mesmo serviço
de interrupções instalado pelo componente; cada ISR só registra timestamp e
contadores inteiros. Nenhum modo imprime na ISR. O serviço de interrupções é
instalado em IRAM e o ciclo de vida do driver roda na tarefa principal/core 0.
`CONFIG_ESP_TIMER_IN_IRAM=y` é obrigatório para timestamp na ISR.

### Consistência temporal das leituras (V3)

As leituras da captura de diagnóstico copiam contadores e timestamps ISR sob a
seção crítica curta, liberam o lock e só então consultam `esp_timer_get_time()`.
O timestamp posterior é guardado junto ao snapshot e usado para idade, timeout,
estados e janelas do T3; o horário recebido antes da atualização do modo não é
usado para avaliar esse snapshot. Assim, uma interrupção já incluída na cópia
nunca fica no futuro em relação ao horário da leitura. Uma interrupção que
ocorra depois da cópia aparece na próxima leitura. T3 e T6, ao usarem
`speed_sensor_read()`, também consultam o relógio depois que a API retorna; a
contagem e o período continuam sendo o snapshot interno do driver. Isso pode
deixar a idade alguns microssegundos mais avançada que o snapshot do driver,
mas não negativa. TIMEOUT continua invalidando período/frequência atuais, e uma
retomada exige dois novos eventos para estabelecer período válido.

## Identificação no boot

O cabeçalho informa identificador e versão do firmware, IDF, modelo/revisão do
chip, motivo do reset, data/hora de compilação e prefixo do SHA-256 do ELF. O
`boot_id` aleatório identifica a inicialização atual; `run_id` muda a cada
seleção/reinício de modo. Não há contador persistente de boots. Caracteres
seriais corrompidos, por si só, não identificam a causa: confira baud, reinícios,
alimentação e o motivo de reset informado.

Monitor serial: 115200 baud. Atualização padrão: 200 ms. O modo inicial é T0;
selecione os demais pelo terminal serial. Cada saída inclui teste, boot, run,
uptime e configuração relevante. `--` indica dado ainda indisponível; TIMEOUT
zera frequência e RPM e não reaproveita o período anterior.

## Comandos seriais

Digite o comando e pressione Enter:

| Comando | Ação |
|---|---|
| `T0` … `T7` | Para o modo atual, limpa os estados e inicia o escolhido |
| `EDGE RISING` | Captura bordas de subida; aplicável a T2, T6 e T7 |
| `EDGE FALLING` | Captura bordas de descida; aplicável a T2, T6 e T7 |
| `EDGE BOTH` | Conta ambas as bordas em T2; não infere direção pela leitura posterior do GPIO |
| `FREQ 1`, `5`, `10`, `20`, `100`, `500` ou `1000` | Reinicia T3 nessa frequência |
| `CAPTURE DIAG` / `CAPTURE DRIVER` | Reinicia T3 com captura própria ou com o driver original |
| `FILTER <us>` | Configura o filtro temporal por software em T7; `0` desliga |
| `PPR <n>` | Configura eventos aceitos por revolução em T7 e revoga a confirmação de RPM |
| `RPM CONFIRM` / `RPM OFF` | Declara/revoga manualmente a confirmação de eventos por volta em T7 |
| `MARK METAL` / `MARK GAP` | Registra marcador manual em T4 |
| `NOTE <texto>` | Registra observação manual do osciloscópio em T5 |
| `HELP` | Lista os comandos |

`RPM CONFIRM` só deve ser usado após comparar os pulsos com uma referência
mecânica independente. Uma frequência estável não prova um pulso por revolução.

## Modos e conexões

Faça as conexões com o ESP32 desenergizado. Use GND comum. GPIOs do ESP32 aceitam
níveis lógicos de aproximadamente 3,3 V; **não aplique 5 V diretamente ao
GPIO27**. Antes de ligar a saída do sensor ao ESP32, meça sinal em relação ao GND
com osciloscópio ou multímetro e confirme amplitude, polaridade e condicionamento.
Se a saída comprovadamente for open-collector compatível, pode ser usado pull-up
externo para 3,3 V. Se for uma saída ativa de 5 V, use adaptação de nível
apropriada. Pull-ups/pull-downs internos não substituem condicionamento externo.

O sensor foi observado experimentalmente com alimentação de 5 V, pino 1 em VCC,
pino 2 em sinal, pino 3 em GND e resistor externo de 1 kΩ. Isso não valida
definitivamente pinagem nem tipo elétrico de saída. A forma de onda deve ser
observada entre sinal e GND. Verifique ruído, amplitude, tempos de subida/descida,
polaridade e transições espúrias.

Para qualquer rotação contínua, fixe sensor, engrenagem e suporte, inspecione a
região usinada após a remoção dos dentes, avalie balanceamento e use proteção
contra partes móveis. Não toque nem reposicione a engrenagem enquanto estiver
girando; pare e desenergize o conjunto antes de qualquer ajuste.

### T0 — Boot e integridade do ESP32

Não configura a entrada do sensor nem gera pulsos. Mostra chip, revisão, reset,
versões, identidade/hash do firmware e heartbeat de um segundo. Serve para
investigar resets e boot/serial sem o sensor conectado. Não há contagem persistida
de boots.

### T1 — GPIO sem sinal

Sensor desconectado; ligue GPIO27 ao GND através de resistor externo de 10 kΩ.
Pull interno é desabilitado pelo modo. Esperado: LEVEL=0, IRQ_TOTAL=0 e
IRQ_RATE_HZ=0. Qualquer interrupção é atividade não explicada; não declare
aprovação até encontrar sua causa. `IRQ_RATE_HZ` é a taxa média na janela
corrente, calculada com o tempo real transcorrido.

### T2 — Entrada digital conhecida

Sensor desconectado. Use GND comum e alterne manualmente GPIO27 entre GND e 3,3 V
através de resistor série de 1 kΩ. Selecione `EDGE RISING`, `EDGE FALLING` ou
`EDGE BOTH`. Uma sequência LOW→HIGH→LOW deve contar, respectivamente, uma, uma
ou duas bordas. A leitura de nível é feita na tarefa; BOTH conta eventos brutos
sem tentar inferir direção dentro da ISR. Não há cálculo de RPM.

### T3 — Gerador interno

Sensor desconectado. Ligue GPIO26 (saída 0–3,3 V) a GPIO27 através de resistor
série de 1 kΩ e compartilhe GND. O periférico LEDC gera uma onda quadrada de 50%
em hardware, sem agendamento de tarefa, em 1, 5, 10, 20, 100, 500 ou 1000 Hz.
GPIO27 captura somente subida. A saída mostra frequência e período medidos,
contagens total e relativa, frequência da janela, erro percentual do período e
eventos esperados/observados. Uma borda de diferença pode ocorrer nas fronteiras
da janela; diferenças maiores aparecem como `MISMATCH_REVIEW_SIGNAL_AND_CAPTURE`.
Os períodos de referência são 1.000.000, 200.000, 100.000, 50.000, 10.000,
2.000 e 1.000 µs, respectivamente. LEDC usa REF_TICK nas frequências até 20 Hz
e APB de 80 MHz nas frequências de 100 Hz ou mais para manter resolução do
divisor. Confira a frequência real na saída com osciloscópio.

Por padrão, `CAPTURE DIAG` usa a rotina ISR de diagnóstico. Para testar o
componente extraído com a mesma fonte conhecida, digite `CAPTURE DRIVER`. O modo
reinicia T3, para a captura anterior e libera o GPIO antes de iniciar o driver
`speed_sensor`. Compare os registros dos dois backends na mesma frequência; são
execuções separadas e não controlam o GPIO ao mesmo tempo. Se a entrada não
acompanhar o LEDC, confira o jumper, níveis, borda e captura antes de investigar
o sensor Hall.

### T4 — Sensor parado e mudança de nível

Use sensor e interface elétrica já verificada, com a engrenagem parada. O teste
captura ambas as bordas e mostra o nível lógico instantâneo, transições,
intervalo entre eventos e idade da última transição. `LEVEL_AGE_SAMPLED_MS` é
uma estimativa por amostragem; não representa a duração real de HIGH/LOW nem a
largura do pulso. Mudanças mais rápidas que a amostragem podem não aparecer
nessa estimativa; a contagem de IRQ preserva os eventos recebidos. Use o
osciloscópio para medir largura e amplitude. Teste as
hipóteses metal=LOW/gap=HIGH e metal=HIGH/gap=LOW, sem presumir nenhuma. Use
`MARK METAL` e `MARK GAP` como anotações do operador. GPIO não mede amplitude em
volts: use o osciloscópio para isso.

No osciloscópio, conecte a ponta entre sinal e GND comum. Medir VCC em relação ao
sinal mostra uma diferença com polaridade aparente invertida e não substitui
sinal em relação ao GND.

### T5 — Forma de onda no osciloscópio com rotação

O firmware não captura nem simula a forma de onda neste modo. Observe primeiro
o sensor com a engrenagem girando e **sem ligar a saída ao ESP32**. Registre nível
alto/baixo, período, largura do pulso, frequência, ruído, transições múltiplas e
comportamento na passagem do gap com `NOTE <texto>`. Compare com uma referência
independente da velocidade ou da posição do gap: uma onda periódica não comprova
um pulso por volta.

### T6 — Captura selecionada com osciloscópio simultâneo

Depois de validar a interface elétrica, conecte o sinal condicionado ao GPIO27 e
meça o mesmo ponto no osciloscópio, sempre em relação ao GND do ESP32. Selecione
uma borda com `EDGE RISING` ou `EDGE FALLING`. O modo usa diretamente o driver
`speed_sensor` extraído e apresenta nível instantâneo, contagem acumulada e por
intervalo, período/frequência entre eventos, idade e timeout de 5000 ms. RPM fica
desabilitado. Rajadas ou intervalos incompatíveis com a rotação ficam visíveis;
compare período do ESP32 com o osciloscópio.

### T7 — Filtro e RPM (somente após compreender o sinal)

Inicia RAW e filtro por software em paralelo; filtro padrão é 0 µs (desligado).
`FILTER <us>` ajusta o intervalo mínimo. RAW conta todas as interrupções;
FILTERED conta somente os eventos aceitos. Trocar o filtro preserva o total
FILTERED e reancora o delta na contagem atual, sem repetir pulsos antigos. A
rejeição temporal não substitui condicionamento elétrico nem prova que o evento
aceito é correto. `RAW_STATE` descreve a captura sem filtro; `FILTERED_STATE`
descreve apenas eventos aceitos e pode mostrar `TIMEOUT` enquanto RAW continua
`MEDINDO`. Períodos antigos são invalidados após timeout ou troca de filtro.

PCNT é uma contagem independente opcional, habilitada por padrão, sem filtro de
glitch; o filtro PCNT começa em 0 ns e só deve ser configurado após inspecionar a
largura do pulso. O firmware estabelece bases relativas após iniciar ambos os
mecanismos, evitando subtrair totais com origens distintas. Ainda pode haver
diferença de uma borda ao coletar as bases ou nas fronteiras da janela; isso é
tolerância de captura, não perda confirmada. `PCNT_ISR_COMPARE` só indica
divergência para investigação se a diferença exceder uma borda e não houver erro
de leitura/overflow. Overflow no acumulador PCNT ou ISR invalida a comparação até
reiniciar T7. Contagem, períodos e estados RAW/FILTERED são exibidos separados.

Estados: `SEM_PULSOS` ainda não recebeu evento nesta sequência; `PRIMEIRO_EVENTO`
recebeu o primeiro evento e aguarda outro após início/timeout; `AGUARDANDO_PERIODO`
tem eventos, mas ainda não há intervalo válido; `MEDINDO` tem período atual;
`TIMEOUT` perdeu atualidade; `INDISPONIVEL`/`ERRO` indicam leitura ou inicialização
sem dado confiável. RAW e FILTERED podem estar em estados diferentes.

RPM só aparece após `RPM CONFIRM`, que representa a declaração do operador de que
o número de eventos selecionados por volta foi verificado com referência
independente. Padrão PPR=1 é uma hipótese para o gap, não uma conclusão. Se for
confirmado, o cálculo usa `frequency_hz = 1e6 / period_us` e
`rpm = frequency_hz * 60 / PPR`. Timeout zera RPM e invalida período/frequência;
após parada, a primeira nova borda reinicia a medição e a segunda fornece período
novo. A contagem RAW permanece acumulada durante o run.

## Critérios de análise

| Modo | Evidência esperada | Interpretação de falha |
|---|---|---|
| T0 | Heartbeat e motivo de reset plausível | Investigar reset, alimentação, console e versão gravada |
| T1 | Zero interrupções com entrada estável em GND | Verificar resistor, cabeamento, GPIO e ruído |
| T2 | 1/1/2 eventos para subida/descida/ambas | Verificar conexão, referência comum e configuração de borda |
| T3 | Frequência/período coerentes com o gerador, contagens compatíveis | Não avançar a conclusões sobre o Hall; separar GPIO/ISR de sensor |
| T4 | Níveis/transições repetíveis com metal e gap | Conferir sensor, geometria, alimentação e forma de onda no osciloscópio |
| T5 | Níveis e pulsos registrados no osciloscópio | Determinar se o sensor comuta dentes, gap ou ambos |
| T6 | Período no ESP32 compatível com período medido no GPIO | Inspecionar borda, ruído, amplitude e rajadas de IRQ |
| T7 | RAW/FILTERED/PCNT comparáveis e RPM só com relação confirmada | Ajuste do filtro só após medir pulsos legítimos e velocidade máxima |

Uma divergência de contagem não prova defeito de software: investigue sensor,
geometria, posição, condicionamento e ruído. A confirmação de um pulso por volta
requer referência independente. Testes digitais e de software não validam a
interface elétrica nem a mecânica.

## Testes de software no computador

Os testes da lógica não usam hardware:

```bash
cd bench/speed_sensor_diagnostics
./tests/run_host_tests.sh
```

Cobrem estados RAW/FILTERED independentes, milhares de bordas brutas com um só
evento filtrado, troca de filtro sem novos pulsos e delta zero, bases relativas,
GPIOs reservados, seleção de borda, período/frequência/RPM, divisor zero,
timeout, retomada e frequências sintéticas até 1000 Hz. O grupo V3 injeta
deterministicamente pulsos antes da cópia do estado e entre a cópia e a leitura
do relógio, usando a mesma função `diag_measurement_read_consistent()` que
alimenta `diag_capture_read()`. Verifica ordem temporal, idade não negativa,
estados válidos, timeout e retomada. Os testes não substituem T1–T7 físicos.

## Ubuntu: configurar, compilar, gravar e monitorar

Carregue ESP-IDF 6.0.2 e entre no projeto:

```bash
source ~/.espressif/v6.0.2/esp-idf/export.sh
cd bench/speed_sensor_diagnostics
idf.py -B /tmp/speed_sensor_diagnostics_build set-target esp32
idf.py -B /tmp/speed_sensor_diagnostics_build menuconfig
idf.py -B /tmp/speed_sensor_diagnostics_build build
```

Em `menuconfig`, confira GPIO27, GPIO26, atualização 200 ms, timeout 5000 ms,
pull interno desabilitado, borda padrão, T0 no boot, frequência e backend do T3,
PCNT opcional e `CONFIG_ESP_TIMER_IN_IRAM=y`. O menu de projeto permite selecionar outro teste
para iniciar; também é possível iniciar em T0 e digitar `T1` … `T7` no monitor.
Os diretórios de build podem ficar em `/tmp` se o caminho do checkout tiver
acentos.

No ESP32 clássico, o firmware bloqueia GPIO6–11 (flash), GPIO0/2/5/12/15
(strapping de boot) e GPIO1/3 (UART0/monitor). GPIO34–39 só aceitam entrada e
não possuem pull interno; GPIO37/38 também não estão expostos no módulo WROOM-32.
Os demais GPIO34–36/39 podem ser usados como entrada apenas com pull desabilitado.
As macros oficiais do ESP-IDF também validam se o pino existe e aceita a direção
solicitada. T3 bloqueia entrada e saída iguais. `INIT_BLOCKED` identifica a razão
e ocorre antes da inicialização dos periféricos.

Identifique a porta antes de gravar:

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Troque `/dev/ttyUSB0` pelo dispositivo confirmado. O comando abaixo grava e
abre o monitor; a gravação é explícita e não foi feita automaticamente durante
o desenvolvimento:

```bash
idf.py -B /tmp/speed_sensor_diagnostics_build -p /dev/ttyUSB0 flash monitor
```

O monitor usa 115200 baud. Encerre com `Ctrl+]`. Se necessário, use primeiro
`idf.py -B /tmp/speed_sensor_diagnostics_build -p /dev/ttyUSB0 monitor` para
observar um firmware que já esteja gravado, sem flash.

Para exibir e registrar a UART em CSV (opcional; requer `pyserial`), instale o
pacote do Ubuntu e use o coletor no lugar do `idf.py monitor`, pois apenas um
programa pode abrir a porta por vez:

```bash
sudo apt install python3-serial
python3 tools/serial_to_csv.py \
  --port /dev/ttyUSB0 --output speed_diagnostics.csv
```

O coletor mostra as linhas no terminal, encaminha comandos digitados ao ESP32 e
salva hora UTC, teste, `boot_id`, `run_id`, uptime e linha serial original. Use
`--no-input` para somente registrar a saída.

Exemplo ilustrativo (não é medição real):

```text
[BOOT] firmware=BAJA_SPEED_DIAGNOSTICS_T0_T7_V3 boot=53a91b72 version=1 ...
[T6] START firmware=... idf=v6.0.2 ... sha256=14f58c0e7a30db12 boot=53a91b72 run=6
[T6] config gpio_in=27 gpio_gen=26 edge=RISING pull=DISABLED interval_ms=200 timeout_ms=5000 ppr=1 filter_us=0 pcnt=1 mode=DRIVER_EDGE_CAPTURE init=OK
[T6] boot=53a91b72 run=6 up_ms=12400 cfg=RPM_DISABLED_PULSE_PER_REV_UNVERIFIED GPIO=27 EDGE=RISING LEVEL=1 IRQ_TOTAL=2 DELTA=1 PERIOD_US=600000 FREQ_EVENT_HZ=1.666667 AGE_MS=120.000 STATE=MEDINDO
[T6] boot=53a91b72 run=6 up_ms=17400 cfg=RPM_DISABLED_PULSE_PER_REV_UNVERIFIED GPIO=27 EDGE=RISING LEVEL=0 IRQ_TOTAL=2 DELTA=0 PERIOD_US=-- FREQ_EVENT_HZ=0.000000 AGE_MS=5120.000 STATE=TIMEOUT
```

## Validação contínua com a engrenagem

1. Com o conjunto parado, confira fixação, proteção, área dos dentes removidos,
   equilíbrio mecânico e distância sensor/engrenagem.
2. Valide primeiro T0, T1 e T2 sem o sensor; depois T3 com sensor desconectado.
3. Observe a saída elétrica do sensor no osciloscópio em T4/T5 antes de conectar
   ao GPIO. Confirme GND comum e níveis seguros.
4. Com o sinal condicionado e simultaneamente observado no osciloscópio, use T6
   na rotação contínua. Compare períodos e procure rajadas ou múltiplas transições.
5. Em T7, mantenha o filtro desligado inicialmente. Registre RAW, FILTERED,
   rejeições e PCNT. Só experimente filtros com largura de pulso e limite de
   rotação conhecidos.
6. Para verificar PPR, compare eventos por volta com tacômetro, encoder ou
   referência mecânica/óptica independente. O gap de dois dentes nos 69 espaços
   originais e os 67 dentes remanescentes não definem por si só os pulsos do
   sensor. Só então configure `PPR` e use `RPM CONFIRM`.
7. Pare o conjunto e espere `TIMEOUT`; frequência e RPM devem zerar. Ao retomar,
   a primeira borda não deve produzir período novo; a segunda deve fornecer uma
   medição recente, sem intervalo da parada.

Não há resultados físicos declarados neste repositório. As medições do sensor,
níveis elétricos, correspondência de eventos por volta e comportamento mecânico
continuam pendentes de ensaio.
