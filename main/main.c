#include "datalogger.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h"

#define MAX_CHAR_SIZE 64
#define MOUNT_POINT CONFIG_MOUNT_POINT

void app_main(void) {
  esp_err_t ret;

  const char mount_point[] = MOUNT_POINT;
  sdmmc_card_t *card;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();

  datalogger_init(&card, &host, mount_point);
  sdmmc_card_print_info(stdout, card);

  const char *car_data = MOUNT_POINT "/car_data.csv";

  char data[MAX_CHAR_SIZE];
  snprintf(data, MAX_CHAR_SIZE,
           "timestamp,rpm,speed1,speed2,pressure1,pressure2\n");
  ret = datalogger_append_to_file(car_data, data);
  if (ret != ESP_OK) {
    return;
  }

  snprintf(data, MAX_CHAR_SIZE, "%lld,%d,%.2f,%.2f,%.2f,%.2f\n",
           esp_timer_get_time() / 1000000, 2500, 20.00, 20.00, 1000.00,
           1000.00);
  ret = datalogger_append_to_file(car_data, data);
  if (ret != ESP_OK) {
    return;
  }

  datalogger_deinit(&card, &host, mount_point);
}
