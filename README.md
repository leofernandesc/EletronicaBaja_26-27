# Eletrônica Baja UEA 2026/2027

Firmware ESP32 (ESP-IDF) para aquisição de GPS, pulsos de velocidade e registro em cartão SD. RPM e pressões em `main/main.c` ainda são **mockados**. `speed1` usa o componente `speed_sensor`; `speed2` fica vazio até a integração de outro sensor.

## Compilação

Com o ESP-IDF instalado e o alvo ESP32 configurado:

```sh
source ~/esp-idf/export.sh
idf.py build
```

Teste do enquadramento NMEA no computador:

```sh
cc -std=c11 -Wall -Wextra -Werror -Icomponents/gps \
  components/gps/nmea_frame.c tests/test_nmea_frame.c -o /tmp/test_nmea_frame
/tmp/test_nmea_frame
```

Teste da medição de velocidade com pulsos sintéticos:

```sh
cc -std=c11 -Wall -Wextra -Werror -Icomponents/speed_sensor \
  components/speed_sensor/speed_measurement.c tests/test_speed_measurement.c \
  -lm -o /tmp/test_speed_measurement
/tmp/test_speed_measurement
```

## Sensor de velocidade

O componente captura pulsos continuamente por interrupções de GPIO. `main/main.c` consulta a leitura atual a cada amostra do carro, sem esperar por pulsos. Cada instância tem GPIO, calibração, borda e temporização próprios. A API e exemplos para adicionar sensores estão em [components/speed_sensor/README.md](components/speed_sensor/README.md).

Em `idf.py menuconfig`, configure **Speed sensor (first instance in main)**. O GPIO padrão é 27, a borda é de subida e o timeout sem pulsos é de 1000 ms. A distância percorrida por pulso vem **sem calibração (0)**: configure o valor em micrômetros depois de definir a montagem. Até lá, os pulsos são capturados e a velocidade fica indisponível.

As colunas `speed1` e `speed2` representam **m/s**. Campo vazio significa leitura indisponível; `0.00` significa ausência de pulsos por pelo menos o timeout, com calibração configurada. A primeira leitura em movimento exige duas bordas, inclusive após uma parada. A ausência de pulsos não distingue parada de sensor desconectado.

O sinal de entrada deve chegar ao ESP32 já condicionado. Pinagem, alimentação e nível de saída da unidade VW Gol G5 ainda precisam ser verificados; não há um circuito confirmado neste repositório.

## Arquivos no SD

O firmware escreve em `/sdcard` (configurável por menuconfig):

- `car_data_v2.csv`: amostras a 5 Hz, com `boot_id`, sequência e tempo desde o boot em milissegundos.
- `gps_data_v2.csv`: uma linha por sentença RMC válida no checksum. Uma posição sem fix aparece com `valido=0` e campos de posição vazios. A altitude vem de GGA somente quando seu horário UTC coincide com RMC e ela chegou há no máximo dois segundos.
- `status_v2.csv`: sequências emitidas e totais acumulados por boot de perdas na fila, falhas de gravação, eventos GPS perdidos, erros de entrada NMEA, falhas de armazenamento e caudas incompletas removidas.

Os arquivos antigos `car_data.csv` e `gps_data.csv` são preservados. Linhas iniciadas por `#` marcam inicializações e remontagens; leitores CSV podem ignorá-las com `comment='#'`. Use `boot_id` e `sequence` para encontrar lacunas e `uptime_ms` para alinhar amostras do carro e GPS. Horário/data UTC do GPS permanecem como campos adicionais.

O firmware continua adquirindo dados quando o SD falha e tenta remontá-lo a cada dois segundos. Os registros que não couberem na fila ou não puderem ser gravados são contados. Os totais aparecem no serial e são gravados no status quando o SD estiver disponível novamente. Um corte de energia pode perder os últimos registros ainda em trânsito; uma linha incompleta é removida no próximo boot e contabilizada como `tail_repairs`. Não há formatação automática do cartão.

## Validação no veículo

Antes de usar os dados em uma prova, verificar com hardware real: fix e perda de fix do GPS, sentenças ausentes e corrompidas, remoção/reinserção do SD, cartão cheio, horas de operação contínua, corte de energia durante gravação e leitura dos três CSVs após cada cenário. A resistência do sistema FAT/SD a um corte de energia não pode ser garantida apenas pelo build ou pelos testes no computador.
