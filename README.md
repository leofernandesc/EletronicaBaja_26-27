# Eletrônica Baja UEA 2026/2027

Firmware ESP32 (ESP-IDF) para aquisição de GPS e registro em cartão SD. Os valores dos sensores do carro em `main/main.c` ainda são **mockados**.

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

## Arquivos no SD

O firmware escreve em `/sdcard` (configurável por menuconfig):

- `car_data_v2.csv`: amostras a 5 Hz, com `boot_id`, sequência e tempo desde o boot em milissegundos.
- `gps_data_v2.csv`: uma linha por sentença RMC válida no checksum. Uma posição sem fix aparece com `valido=0` e campos de posição vazios. A altitude vem de GGA somente quando seu horário UTC coincide com RMC e ela chegou há no máximo dois segundos.
- `status_v2.csv`: sequências emitidas e totais acumulados por boot de perdas na fila, falhas de gravação, eventos GPS perdidos, erros de entrada NMEA, falhas de armazenamento e caudas incompletas removidas.

Os arquivos antigos `car_data.csv` e `gps_data.csv` são preservados. Linhas iniciadas por `#` marcam inicializações e remontagens; leitores CSV podem ignorá-las com `comment='#'`. Use `boot_id` e `sequence` para encontrar lacunas e `uptime_ms` para alinhar amostras do carro e GPS. Horário/data UTC do GPS permanecem como campos adicionais.

O firmware continua adquirindo dados quando o SD falha e tenta remontá-lo a cada dois segundos. Os registros que não couberem na fila ou não puderem ser gravados são contados. Os totais aparecem no serial e são gravados no status quando o SD estiver disponível novamente. Um corte de energia pode perder os últimos registros ainda em trânsito; uma linha incompleta é removida no próximo boot e contabilizada como `tail_repairs`. Não há formatação automática do cartão.

## Validação no veículo

Antes de usar os dados em uma prova, verificar com hardware real: fix e perda de fix do GPS, sentenças ausentes e corrompidas, remoção/reinserção do SD, cartão cheio, horas de operação contínua, corte de energia durante gravação e leitura dos três CSVs após cada cenário. A resistência do sistema FAT/SD a um corte de energia não pode ser garantida apenas pelo build ou pelos testes no computador.
