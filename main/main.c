#include "datalogger.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gps.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include <sys/stat.h>

#define MAX_CHAR_SIZE 128
#define MOUNT_POINT CONFIG_MOUNT_POINT
#define YEAR_BASE (2000) // date in GPS starts from 2000

#define CAR_DATA_PATH MOUNT_POINT "/car_data.csv"
#define GPS_DATA_PATH MOUNT_POINT "/gps_data.csv"

#define LOG_QUEUE_LENGTH 32      // registros aguardando o cartão SD
#define CAR_SAMPLE_PERIOD_MS 200 // período de aquisição dos sensores (5 Hz)
#define LOGGER_TASK_STACK_SIZE 4096
#define LOGGER_TASK_PRIORITY 3

static const char *TAG = "main";

// Destino de cada registro enfileirado
typedef enum { LOG_TARGET_CAR, LOG_TARGET_GPS } log_target_t;

// Registro pronto para gravação: destino + linha já formatada
typedef struct {
  log_target_t target;
  char line[MAX_CHAR_SIZE];
} log_record_t;

static QueueHandle_t s_log_queue;

// Grava o cabeçalho somente se o arquivo não existir ou estiver vazio: assim
// ele aparece uma única vez, no início do arquivo
static void write_header_if_empty(const char *path, const char *header) {
  struct stat st;
  if (stat(path, &st) == 0 && st.st_size > 0) {
    return;
  }
  datalogger_append_to_file(path, header);
}

// Marca o início de uma nova sessão. Começa com '#' para que leitores como o
// pandas possam ignorar a linha usando comment='#'
static void write_boot_separator(const char *path) {
  datalogger_append_to_file(path, "# ---- boot ----\n");
}

// Enfileira uma linha sem bloquear quem a produziu (parser do GPS ou loop de
// aquisição dos sensores)
static void log_enqueue(log_target_t target, const char *line) {
  log_record_t record;
  record.target = target;
  snprintf(record.line, sizeof(record.line), "%s", line);
  if (xQueueSend(s_log_queue, &record, 0) != pdTRUE) {
    ESP_LOGW(TAG, "Fila de log cheia, registro descartado");
  }
}

// Única task que acessa o cartão SD
static void logger_task(void *arg) {
  log_record_t record;
  while (1) {
    if (xQueueReceive(s_log_queue, &record, portMAX_DELAY) == pdTRUE) {
      const char *path =
          (record.target == LOG_TARGET_GPS) ? GPS_DATA_PATH : CAR_DATA_PATH;
      datalogger_append_to_file(path, record.line);
    }
  }
}

// Leitura dos sensores do carro.
// Por enquanto valores mockados: substituir pelos drivers reais (ADC, CAN,
// etc.) mantendo esta assinatura.
static void read_car_sensors(int *rpm, float *speed1, float *speed2,
                             float *pressure1, float *pressure2) {
  *rpm = 2500;
  *speed1 = 20.00f;
  *speed2 = 20.00f;
  *pressure1 = 1000.00f;
  *pressure2 = 1000.00f;
}

static void gps_event_handler(void *event_handler_arg,
                              esp_event_base_t event_base, int32_t event_id,
                              void *event_data) {
  gps_t *gps = NULL;
  switch (event_id) {
  case GPS_UPDATE: {
    char data[MAX_CHAR_SIZE];
    gps = (gps_t *)event_data;
    /* print information parsed from GPS statements */
    snprintf(data, sizeof(data),
             "%02d:%02d:%02d,%02d/%02d/%04d,%.6f,%.6f,%.2f,%.2f,%d\n",
             gps->tim.hour, gps->tim.minute, gps->tim.second, gps->date.day,
             gps->date.month, gps->date.year + YEAR_BASE, gps->latitude,
             gps->longitude, gps->altitude, gps->speed, gps->valid);
    log_enqueue(LOG_TARGET_GPS, data);
    ESP_LOGI(TAG, "gps data:%s", data);
    break;
  }
  case GPS_UNKNOWN:
    /* print unknown statements */
    ESP_LOGW(TAG, "Sentença NMEA desconhecida recebida:%s", (char *)event_data);
    break;
  default:
    break;
  }
}

void app_main(void) {
  esp_err_t ret;

  const char mount_point[] = MOUNT_POINT;
  sdmmc_card_t *card;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();

  ret = datalogger_init(&card, &host, mount_point);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao inicializar o datalogger: %s", esp_err_to_name(ret));
    return;
  }
  sdmmc_card_print_info(stdout, card);

  write_header_if_empty(
      CAR_DATA_PATH, "timestamp,rpm,speed1,speed2,pressure1,pressure2\n");
  write_header_if_empty(
      GPS_DATA_PATH,
      "hora_utc,data,latitude,longitude,altitude_m,velocidade_mps,valido\n");
  write_boot_separator(CAR_DATA_PATH);
  write_boot_separator(GPS_DATA_PATH);

  s_log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(log_record_t));
  if (s_log_queue == NULL) {
    ESP_LOGE(TAG, "Falha ao criar a fila de log");
    return;
  }
  if (xTaskCreate(logger_task, "sd_logger", LOGGER_TASK_STACK_SIZE, NULL,
                  LOGGER_TASK_PRIORITY, NULL) != pdTRUE) {
    ESP_LOGE(TAG, "Falha ao criar a task de gravação");
    return;
  }

  // Configuracoes GPS

  /* NMEA parser configuration */
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  /* init NMEA parser library */
  nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
  if (nmea_hdl == NULL) {
    ESP_LOGE(TAG, "Falha ao inicializar o parser NMEA");
    return;
  }
  /* register event handler for NMEA parser library */
  nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

  // Aquisição contínua: o parser NMEA e a task de gravação seguem rodando
  // indefinidamente (nada de deinit/unmount aqui)
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    int rpm;
    float speed1, speed2, pressure1, pressure2;
    char data[MAX_CHAR_SIZE];

    read_car_sensors(&rpm, &speed1, &speed2, &pressure1, &pressure2);
    snprintf(data, sizeof(data), "%lld,%d,%.2f,%.2f,%.2f,%.2f\n",
             esp_timer_get_time() / 1000, rpm, speed1, speed2, pressure1,
             pressure2);
    log_enqueue(LOG_TARGET_CAR, data);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAR_SAMPLE_PERIOD_MS));
  }
}
