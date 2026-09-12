#include "datalogger.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "sdmmc_cmd.h"
#include <stdio.h>

#define MAX_CHAR_SIZE 128
#define MOUNT_POINT CONFIG_MOUNT_POINT
#define YEAR_BASE (2000) // date in GPS starts from 2000

static const char *TAG = "main";

static void gps_event_handler(void *event_handler_arg,
                              esp_event_base_t event_base, int32_t event_id,
                              void *event_data) {
  gps_t *gps = NULL;
  switch (event_id) {
  case GPS_UPDATE:
    char data[MAX_CHAR_SIZE];
    gps = (gps_t *)event_data;
    /* print information parsed from GPS statements */
    snprintf(data, sizeof(data),
             "%02d:%02d:%02d,%02d/%02d/%04d,%.6f,%.6f,%.2f,%.2f,%d\n",
             gps->tim.hour, gps->tim.minute, gps->tim.second, gps->date.day,
             gps->date.month, gps->date.year + YEAR_BASE, gps->latitude,
             gps->longitude, gps->altitude, gps->speed, gps->valid);
    datalogger_append_to_file(MOUNT_POINT "/gps_data.csv", data);
    ESP_LOGI(TAG, "gps data:%s", data);
    break;
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

  datalogger_init(&card, &host, mount_point);
  sdmmc_card_print_info(stdout, card);

  const char *car_data = MOUNT_POINT "/car_data.csv";
  const char *gps_data = MOUNT_POINT "/gps_data.csv";

  char header[MAX_CHAR_SIZE]; // Fiz a divisão entre header e data
  snprintf(header, MAX_CHAR_SIZE,
           "timestamp,rpm,speed1,speed2,pressure1,pressure2\n");
  ret = datalogger_append_to_file(car_data, header);
  if (ret != ESP_OK) {
    return;
  }
  snprintf(
      header, MAX_CHAR_SIZE,
      "hora_utc,data,latitude,longitude,altitude_m,velocidade_mps,valido\n");
  ret = datalogger_append_to_file(gps_data, header);
  if (ret != ESP_OK) {
    return;
  }

  // Configuracoes GPS

  /* NMEA parser configuration */
  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  /* init NMEA parser library */
  nmea_parser_handle_t nmea_hdl = nmea_parser_init(&config);
  /* register event handler for NMEA parser library */
  nmea_parser_add_handler(nmea_hdl, gps_event_handler, NULL);

  // Tratamento dos dados mockados

  char data[MAX_CHAR_SIZE];
  snprintf(data, MAX_CHAR_SIZE, "%lld,%d,%.2f,%.2f,%.2f,%.2f\n",
           esp_timer_get_time() / 1000000, 2500, 20.00, 20.00, 1000.00,
           1000.00);
  ret = datalogger_append_to_file(car_data, data);
  if (ret != ESP_OK) {
    return;
  }

  vTaskDelay(10000 /
             portTICK_PERIOD_MS); // Teste para funcionamento do GPS durante 10
                                  // segundos, não seria necessário para ele
                                  // funcionar continuamente

  /* unregister event handler */
  nmea_parser_remove_handler(nmea_hdl, gps_event_handler);
  /* deinit NMEA parser library */
  nmea_parser_deinit(nmea_hdl);

  datalogger_deinit(&card, &host, mount_point);
}
