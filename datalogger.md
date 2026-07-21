Fazendo isso pra documentar o processo de fazer o datalogger, acho que vai ser
bom pra compartilhar e estudar depois

Eu sabia que eles tinham alguns exemplos que daria pra usar e encontrei um de
cartão SD, o `$IDF_PATH/examples/storage/sd_card/sdspi`.

O código desse exemplo usa a função `esp_vfs_fat_sdspi_mount` pra inicializar a
conexão com o módulo do cartão SD através do SPI e montar o cartão SD no
sistema. Ele imprime algumas informações sobre o cartão, cria um arquivo com
`fopen` e escreve no arquivo usando `fprintf`. Aí ele muda o nome do arquivo, mas
antes verifica se já existe um arquivo com o mesmo nome com `stat` e remove ele
com `unlink`. Por fim, ele abre o arquivo e imprime o conteúdo dele. Também tem a
opção de formatar o cartão SD, mas ela é opcional. Isso tudo tá escrito na
documentação do exemplo.

Fazendo a leitura do código pra ver como ele funciona, descobri o `#if`e o `#ifdef`.
A sintaxe utilizada é `#if expression` e se o resultado da expressão for diferente de
0 o código dentro dele, que termina em `#endif` é processado pelo compilador.
Caso contrário, o compilador vai pular esse código e não vai incluí-lo no binário
final. O `#ifdef` funciona de forma semelhante mas, ao invés de verificar o
resultado de uma expressão, verifica se algum macro foi definido com `#define`.
Essa abordagem é bem interessante pra reduzir o tamanho do binário final, e como
trabalhamos com um sistema embarcado, com bem menos espaço que um
computador normal, pode ser muito útil. O código usa essa ferramenta pra Debug,
ainda não entendi muito bem como isso é feito mas pode ser útil pra gente.

Uma das primeiras partes do código é esse bloco, que define os pinos utilizados no
protocolo SPI:
```
#define PIN_NUM_MISO  CONFIG_EXAMPLE_PIN_MISO
#define PIN_NUM_MOSI  CONFIG_EXAMPLE_PIN_MOSI
#define PIN_NUM_CLK   CONFIG_EXAMPLE_PIN_CLK
#define PIN_NUM_CS    CONFIG_EXAMPLE_PIN_CS
```
As constantes que começam com `CONFIG_EXAMPLE` são definidas através do menu,
com `idf.py menuconfig`. Pode ser muito interessante a gente estudar isso pra
aplicar no projeto.

Logo depois temos a função `s_example_write_file`, que é a responsável por
escrever coisas no arquivo. Ela recebe os parâmetros `path` e `data`, strings
indicando o caminho do arquivo e os dados a serem inseridos, respectivamente.
Ela usa a função `fopen` do `<stdio.h>` no modo write, de escrita. Pro nosso caso,
que vamos fazer um datalogger, talvez seja mais interessante abrirmos o arquivo no
modo append, pra colocar conteúdo novo no final do arquivo ao invés de substituir
o que já existe.

Temos também a função `s_example_read_file`, que lê a primeira linha do arquivo.
Pro nosso caso, não sei se ela vai ser muito útil, já que vamos provavelmente abrir
o arquivo gerado em algum aplicativo ao invés do terminal.

Agora chegamos na parte principal do código, que é onde a nossa aplicação roda,
dentro de `app_main(void)`. Logo no começo, temos o seguinte bloco de código:
```c
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
```
Achei meio estranho no começo mas fui lendo com calma e olhando a
documentação do framework e consegui tirar algumas coisas.

A primeira é que a maior parte das configurações, inicializações e instalações do
ESP-IDF são feitas com um ponteiro que aponta para um struct contendo as
configurações. Nesse caso, temos um struct do tipo
`esp_vfs_fat_sdmmc_mount_config_t` que contém os membros
`format_if_mount_failed`, `max_files` e `allocation_unit_size`. O membro
interessante aqui é o `format_if_mount_failed`, que é definido duas vezes no
código. Isso acontece porque ele é definido dentro de um bloco `#ifdef`, que
verifica se o macro de configuração `CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
está definido e, se estiver, define `format_if_mount_failed` como `true`, caso
contrário, como `false`. É uma abordagem interessante, presente em quase todos
os exemplos dentro do ESP-IDF, que aplica um pouco do conceito que expliquei
antes.

Depois disso é a configuração do protocolo SPI e a parte de escrita e leitura dos
arquivos.

Acredito que a maior parte do código vá ser reutilizada, mudando somente a
questão da escrita e da leitura, que expliquei antes. A minha ideia agora é fazer um
código modular que vamos conseguir utilizar com qualquer sensor, módulo etc. Por
isso, o ideal nesse momento é definir a estrutura dos dados que vão ser salvos e,
talvez, do sistema no geral.

Antes de tudo, a base: acredito que a melhor forma de estruturar esses dados é em
um arquivo .csv, que podemos abrir tanto como tabela quanto como texto, além de
ser muito fácil de trabalhar em outras linguagens de programação, como Python ou
Javascript, na hora de fazer uma visualização da telemetria. Sendo assim, o que
vou definir aqui é a estrutura de uma tabela.

Pensei nas seguintes abordagens:
1. Um único arquivo com uma coluna indicando o tempo e várias outras colunas, cada uma indicando a leitura dos módulos nesse tempo
2. Um único arquivo com uma coluna indicando o tempo, o módulo lido e a leitura do módulo
3. Vários arquivos, um para cada módulo, com o tempo e a leitura do módulo
Cada uma tem suas vantagens e desvantagens, que vou explicar agora.

A primeira abordagem tem a vantagem de conseguir comparar os estados de
diferentes módulos em um mesmo instante. A segunda abordagem tem a
vantagem de conseguir ver os acontecimentos como se fosse uma "linha do
tempo". A desvantagem das duas, porém, é que com um único arquivo grande
pode ser difícil analisar uma variável individualmente, mas isso pode ser
contornado facilmente. A terceira abordagem tem a vantagem de conseguir
analisar cada variável separadamente, mas tem duas desvantagens que pesam:
com vários arquivos, a transferência de dados vai ficar lenta e em um contexto de
telemetria provavelmente não iria refletir o estado atual do carro, e, pelo mesmo
motivo, não dá para comparar o estado do carro inteiro sem fazer o "cruzamento"
dos dados.

Com base nessa análise, acredito que a melhor abordagem aqui é a primeira, com
um único arquivo e uma coluna para cada módulo. A tabela seria algo semelhante
a essa aqui:

| timestamp          | rpm  | speed1 | speed2 | pressure1 | pressure2 |
| ------------------ | ---- | ------ | ------ | --------- | --------- |
| 07-19 15:29:42.067 | 2000 | 23.32  | 23.31  | 200       | 200       |
O arquivo em csv dessa tabela seria assim:
```
timestamp,rpm,speed1,speed2,pressure1,pressure2
"07-19 15:29:42.067",2000,23.32,23.31,200,200
```

Na minha visão é algo simples, leve e fácil de implementar. Por isso, vou começar
a implementação agora.

O primeiro passo seria modificar a função de escrita existente para ela usar o
modo append ao invés de write. Atualmente ela é assim:
```c
static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}
```
A mudança é muito simples, vamos só trocar o `w` do `fopen` para `a`. Ficou assim:
```c
static esp_err_t s_example_write_file(const char *path, char *data) {
  ESP_LOGI(TAG, "Opening file %s", path);
  FILE *f = fopen(path, "a");
  if (f == NULL) {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return ESP_FAIL;
  }
  fprintf(f, data);
  fclose(f);
  ESP_LOGI(TAG, "File written");

  return ESP_OK;
}
```

O próximo passo é estruturar como cada linha vai ficar no arquivo. A primeira
linha é, logicamente, o cabeçalho da tabela, como mostrado antes. O arquivo já
tem vários exemplos de texto sendo escrito em um arquivo, então só copiei e
modifiquei um deles:
```c
  const char *car_data = MOUNT_POINT "/car_data.csv";
  char data[EXAMPLE_MAX_CHAR_SIZE];
  snprintf(data, EXAMPLE_MAX_CHAR_SIZE,
           "timestamp,rpm,speed1,speed2,pressure1,pressure2\n");
  ret = s_example_write_file(car_data, data);
  if (ret != ESP_OK) {
    return;
  }
```

Agora vamos adicionar conteúdo ao arquivo. Nesse momento a medição ainda não
é real, mas eu quero estruturar o código como se fosse.

Como quero fazer dessa maneira, preciso saber como funciona o sistema de builds do ESP-IDF e o conceito de "componentes". Para isso, a documentação do
framework define alguns conceitos básicos:
- Um **projeto** é um diretório que contém todos os arquivos de um **app**.
- A **Configuração do Projeto** é colocada em um único arquivo chamado `sdkconfig` na raiz do projeto. Essa configuração é modificada através do comando `idf.py menuconfig`.
- Um **app** é o executável construído pelo ESP-IDF. Um único projeto geralmente constrói 2 apps: o app de projeto (o nosso código) e um bootloader.
- **Componentes** são módulos de códigos independentes que são compilados em bibliotecas estáticas. Alguns são do próprio ESP-IDF, outros podem ser encontrados na internet ou criados do zero.
- **Target** é o hardware em que o **app** vai rodar.
Com isso fixado, podemos partir para o funcionamento do sistema de builds. Esse
sistema utiliza 3 ferramentas principais para gerenciar as builds dos projetos: o
CMake (configuração do projeto), o Ninja (compilação da build) e o esptool (flash
da build). A ferramenta `idf.py` é um "front-end" que organiza todas essas outras
ferramentas e deixa mais fácil a organização do projeto.

Uma organização comum das pastas de um projeto é assim:
```
projeto/

├── components/
│   │
│   ├── display/
│   │     ├── display.c
│   │     ├── display.h
│   │     ├── Kconfig
│   │     └── CMakeLists.txt
│   │
│   └── sensor/
│         ├── sensor.c
│         ├── sensor.h
│         ├── Kconfig
│         └── CMakeLists.txt
│
├── main/
│     ├── main.c
│     └── CMakeLists.txt
│
├── CMakeLists.txt
└── sdkconfig
```

Todo projeto tem um arquivo `CMakeLists.txt` na raiz que contém as
configurações de build do projeto inteiro. Um `CMakeLists.txt` comum é assim:
```cmake
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bajaUEA)
```
Essas 3 linhas são necessárias para qualquer projeto do ESP-IDF:
- `cmake_minimum_required(VERSION 3.22)`: versão mínima do CMake para o projeto
- `include($ENV{IDF_PATH}/tools/cmake/project.cmake)`: inclui o CMake no projeto, pegando ele do próprio ESP-IDF
- `project(bajaUEA)`: cria o projeto, especificando o nome dele.

Cada componente também tem um arquivo `CMakeLists.txt` na raiz. Um arquivo
exemplo de configuração do componente é assim:
```cmake
idf_component_register(
	SRCS
		"datalogger.c"
		"datalogger.h"
	INCLUDE_DIRS
		"."
	REQUIRES
		fatfs
		sd_card
)
```
- `SRCS` é a lista de arquivos do componente
- `INCLUDE_DIRS`é a lista de diretórios do componente
- `REQUIRES` é a lista de dependências do componente, não é obrigatório mas a maioria depende de outros componentes.

Um componente também pode ter um arquivo `Kconfig` contendo as
configurações para serem adicionadas ao menu (acessado com
`idf.py menuconfig`). Um uso comum para esse arquivo é a definição de pinos
utilizados no projeto. No datalogger, por exemplo:
```cmake
menu "Pinos do módulo de cartão SD"
w	config PIN_CLK
		int "Número do pino GPIO CLK"
		default 18
	config PIN_CS
		int "Número do pino GPIO CS"
		default 5
	config PIN_MISO
		int "Número do pino GPIO MISO"
		default 19
	config PIN_MOSI
		int "Número do pino GPIO MOSI"
		default 23
endmenu
```

 Quando rodamos o comando `idf.py menuconfig`, ele cria um arquivo chamado
 `sdkconfig` contendo todas as configurações do projeto, incluindo essa que
 acabamos de criar. O mesmo arquivo é criado em diversos formatos, incluindo
 `sdkconfig.h`. Com isso, podemos chamar esses valores dentro de um código C.
 Isso é feito dentro do exemplo que estávamos utilizando, e podemos adaptar
 para a nossa configuração:
 ```
#include "sdkconfig.h"
(...)
#define PIN_NUM_MISO CONFIG_PIN_MISO
#define PIN_NUM_MOSI CONFIG_PIN_MOSI
#define PIN_NUM_CLK CONFIG_PIN_CLK
#define PIN_NUM_CS CONFIG_PIN_CS
 ```

 Com isso, podemos fazer uma configuração básica do projeto seguindo esses
 passos:
 1. Criar uma pasta para o projeto
 2. Criar a configuração do projeto com `CMakeLists.txt`
 3. Criar a pasta main
 4. Criar a configuração do componente main
 5. Criar o arquivo main.c
 6. OPCIONAL: Criar o arquivo `sdkconfig`
 7. Repetir 3-6 para cada componente

Acabei me empolgando e fiz quase o projeto todo sem documentar, então o que
vem agora é eu tentando lembrar o que fiz ou uma visão geral do projeto.

FALTA CRIAR UM MOCK DOS SENSORES