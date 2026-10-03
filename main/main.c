#include "datalogger.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gps.h"
#include "sdmmc_cmd.h"
#include <stdint.h>
#include <stdio.h>

#define MAX_CHAR_SIZE 192
#define MOUNT_POINT CONFIG_MOUNT_POINT
#define YEAR_BASE 2000
#define CAR_DATA_PATH MOUNT_POINT "/car_data_v2.csv"
#define GPS_DATA_PATH MOUNT_POINT "/gps_data_v2.csv"
#define STATUS_PATH MOUNT_POINT "/status_v2.csv"
#define LOG_QUEUE_LENGTH 32
#define CAR_SAMPLE_PERIOD_MS 200
#define STORAGE_RETRY_US 2000000
#define STATUS_PERIOD_US 1000000
#define LOGGER_TASK_STACK_SIZE 4096
#define LOGGER_TASK_PRIORITY 1
#define CAR_TASK_PRIORITY 3

static const char *TAG = "main";
typedef enum { LOG_TARGET_CAR, LOG_TARGET_GPS } log_target_t;
typedef struct {
  log_target_t target;
  char line[MAX_CHAR_SIZE];
} log_record_t;

static QueueHandle_t s_log_queue;
static nmea_parser_handle_t s_gps_handle;
static uint32_t s_boot_id;
static uint32_t s_car_sequence;
static uint32_t s_gps_sequence;
static uint32_t s_queue_car_lost;
static uint32_t s_queue_gps_lost;
static uint32_t s_write_car_failed;
static uint32_t s_write_gps_failed;
static uint32_t s_storage_failures;
static uint32_t s_tail_repairs;

static uint32_t counter_get(uint32_t *counter) {
  return __atomic_load_n(counter, __ATOMIC_RELAXED);
}

static void counter_add(uint32_t *counter) {
  __atomic_fetch_add(counter, 1, __ATOMIC_RELAXED);
}

static void log_enqueue(log_target_t target, const char *line) {
  log_record_t record = {.target = target};
  int written = snprintf(record.line, sizeof(record.line), "%s", line);
  if (written < 0 || written >= sizeof(record.line) ||
      xQueueSend(s_log_queue, &record, 0) != pdTRUE) {
    uint32_t count = __atomic_add_fetch(
        target == LOG_TARGET_GPS ? &s_queue_gps_lost : &s_queue_car_lost,
        1, __ATOMIC_RELAXED);
    if (count == 1 || count % 100 == 0)
      ESP_LOGW(TAG, "Records dropped before SD queue: %lu",
               (unsigned long)count);
  }
}

static esp_err_t prepare_logs(void) {
  bool trimmed = false;
  if (datalogger_prepare_file(
          CAR_DATA_PATH,
          "boot_id,sequence,uptime_ms,rpm,speed1,speed2,pressure1,pressure2\n",
          &trimmed) != ESP_OK) return ESP_FAIL;
  if (trimmed) counter_add(&s_tail_repairs);
  if (datalogger_prepare_file(
          GPS_DATA_PATH,
          "boot_id,sequence,uptime_ms,hora_utc,data,latitude,longitude,altitude_m,velocidade_mps,valido\n",
          &trimmed) != ESP_OK) return ESP_FAIL;
  if (trimmed) counter_add(&s_tail_repairs);
  if (datalogger_prepare_file(
          STATUS_PATH,
          "boot_id,uptime_ms,car_issued,gps_issued,queue_car_lost,queue_gps_lost,write_car_failed,write_gps_failed,gps_event_lost,gps_input_errors,storage_failures,tail_repairs\n",
          &trimmed) != ESP_OK) return ESP_FAIL;
  if (trimmed) counter_add(&s_tail_repairs);

  char marker[80];
  snprintf(marker, sizeof(marker), "# boot_id=%08lx reset_reason=%d\n",
           (unsigned long)s_boot_id, (int)esp_reset_reason());
  if (datalogger_append_to_file(CAR_DATA_PATH, marker) != ESP_OK ||
      datalogger_append_to_file(GPS_DATA_PATH, marker) != ESP_OK ||
      datalogger_append_to_file(STATUS_PATH, marker) != ESP_OK) return ESP_FAIL;
  return ESP_OK;
}

static esp_err_t write_status(void) {
  nmea_parser_stats_t gps_stats = {0};
  nmea_parser_handle_t gps_handle =
      __atomic_load_n(&s_gps_handle, __ATOMIC_ACQUIRE);
  if (gps_handle) nmea_parser_get_stats(gps_handle, &gps_stats);
  char line[MAX_CHAR_SIZE];
  int written = snprintf(
      line, sizeof(line), "%08lx,%lld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
      (unsigned long)s_boot_id, (long long)(esp_timer_get_time() / 1000),
      (unsigned long)counter_get(&s_car_sequence),
      (unsigned long)counter_get(&s_gps_sequence),
      (unsigned long)counter_get(&s_queue_car_lost),
      (unsigned long)counter_get(&s_queue_gps_lost),
      (unsigned long)counter_get(&s_write_car_failed),
      (unsigned long)counter_get(&s_write_gps_failed),
      (unsigned long)gps_stats.dropped_events,
      (unsigned long)gps_stats.input_errors,
      (unsigned long)counter_get(&s_storage_failures),
      (unsigned long)counter_get(&s_tail_repairs));
  if (written < 0 || written >= sizeof(line)) return ESP_FAIL;
  return datalogger_append_to_file(STATUS_PATH, line);
}

static void logger_task(void *arg) {
  sdmmc_card_t *card = NULL;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  bool mounted = false;
  int64_t retry_after_us = 0;
  int64_t next_status_us = 0;
  int64_t next_report_us = 0;
  log_record_t record;
  while (1) {
    int64_t now = esp_timer_get_time();
    if (now >= next_report_us) {
      nmea_parser_stats_t gps_stats = {0};
      nmea_parser_handle_t gps_handle =
          __atomic_load_n(&s_gps_handle, __ATOMIC_ACQUIRE);
      if (gps_handle) nmea_parser_get_stats(gps_handle, &gps_stats);
      if (counter_get(&s_queue_car_lost) || counter_get(&s_queue_gps_lost) ||
          counter_get(&s_write_car_failed) || counter_get(&s_write_gps_failed) ||
          counter_get(&s_storage_failures) || counter_get(&s_tail_repairs) ||
          gps_stats.dropped_events || gps_stats.input_errors)
        ESP_LOGW(TAG, "Loss totals queue car=%lu gps=%lu, write car=%lu gps=%lu, GPS event=%lu frame=%lu, storage=%lu tail=%lu",
                 (unsigned long)counter_get(&s_queue_car_lost),
                 (unsigned long)counter_get(&s_queue_gps_lost),
                 (unsigned long)counter_get(&s_write_car_failed),
                 (unsigned long)counter_get(&s_write_gps_failed),
                 (unsigned long)gps_stats.dropped_events,
                 (unsigned long)gps_stats.input_errors,
                 (unsigned long)counter_get(&s_storage_failures),
                 (unsigned long)counter_get(&s_tail_repairs));
      next_report_us = now + 5000000;
    }
    if (!mounted && now >= retry_after_us) {
      esp_err_t err = ESP_OK;
      if (card) err = datalogger_deinit(&card, &host, MOUNT_POINT);
      if (err == ESP_OK) err = datalogger_init(&card, &host, MOUNT_POINT);
      if (err == ESP_OK) err = prepare_logs();
      if (err == ESP_OK) {
        mounted = true;
        next_status_us = 0;
        ESP_LOGI(TAG, "SD logging available");
      } else {
        counter_add(&s_storage_failures);
        ESP_LOGE(TAG, "SD unavailable: %s", esp_err_to_name(err));
        if (card) datalogger_deinit(&card, &host, MOUNT_POINT);
        retry_after_us = now + STORAGE_RETRY_US;
      }
    }

    if (mounted && now >= next_status_us) {
      if (write_status() != ESP_OK) {
        counter_add(&s_storage_failures);
        ESP_LOGE(TAG, "Status write failed; retrying SD mount");
        mounted = false;
        if (card) datalogger_deinit(&card, &host, MOUNT_POINT);
        retry_after_us = now + STORAGE_RETRY_US;
      }
      next_status_us = now + STATUS_PERIOD_US;
    }

    if (xQueueReceive(s_log_queue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
      const char *path = record.target == LOG_TARGET_GPS ? GPS_DATA_PATH : CAR_DATA_PATH;
      if (!mounted || datalogger_append_to_file(path, record.line) != ESP_OK) {
        counter_add(record.target == LOG_TARGET_GPS ? &s_write_gps_failed
                                                    : &s_write_car_failed);
        if (mounted) {
          ESP_LOGE(TAG, "Record not stored (%s); retrying SD mount", path);
          counter_add(&s_storage_failures);
          mounted = false;
          if (card) datalogger_deinit(&card, &host, MOUNT_POINT);
          retry_after_us = esp_timer_get_time() + STORAGE_RETRY_US;
        }
      }
    }
  }
}

// Replace these deliberate placeholders with the vehicle's sensor drivers.
static void read_car_sensors(int *rpm, float *speed1, float *speed2,
                             float *pressure1, float *pressure2) {
  *rpm = 2500;
  *speed1 = 20.00f;
  *speed2 = 20.00f;
  *pressure1 = 1000.00f;
  *pressure2 = 1000.00f;
}

static void car_task(void *arg) {
  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    int rpm;
    float speed1, speed2, pressure1, pressure2;
    char line[MAX_CHAR_SIZE];
    read_car_sensors(&rpm, &speed1, &speed2, &pressure1, &pressure2);
    uint32_t sequence = __atomic_add_fetch(&s_car_sequence, 1, __ATOMIC_RELAXED);
    int written = snprintf(
        line, sizeof(line), "%08lx,%lu,%lld,%d,%.2f,%.2f,%.2f,%.2f\n",
        (unsigned long)s_boot_id, (unsigned long)sequence,
        (long long)(esp_timer_get_time() / 1000), rpm, speed1, speed2,
        pressure1, pressure2);
    if (written >= 0 && written < sizeof(line))
      log_enqueue(LOG_TARGET_CAR, line);
    else
      counter_add(&s_queue_car_lost);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAR_SAMPLE_PERIOD_MS));
  }
}

static void gps_event_handler(void *arg, esp_event_base_t base,
                              int32_t event_id, void *event_data) {
  if (event_id == GPS_UNKNOWN) return;
  if (event_id != GPS_UPDATE || !event_data) return;
  const gps_t *gps = (const gps_t *)event_data;
  char latitude[24] = "", longitude[24] = "", altitude[48] = "", speed[48] = "";
  if (gps->valid) {
    snprintf(latitude, sizeof(latitude), "%.6f", gps->latitude);
    snprintf(longitude, sizeof(longitude), "%.6f", gps->longitude);
    if (gps->speed_valid)
      snprintf(speed, sizeof(speed), "%.2f", gps->speed);
    if (gps->altitude_valid)
      snprintf(altitude, sizeof(altitude), "%.2f", gps->altitude);
  }
  char time[16] = "", date[16] = "";
  if (gps->time_valid)
    snprintf(time, sizeof(time), "%02u:%02u:%02u", gps->tim.hour,
             gps->tim.minute, gps->tim.second);
  if (gps->date_valid)
    snprintf(date, sizeof(date), "%02u/%02u/%04u", gps->date.day,
             gps->date.month, gps->date.year + YEAR_BASE);
  uint32_t sequence = __atomic_add_fetch(&s_gps_sequence, 1, __ATOMIC_RELAXED);
  char line[MAX_CHAR_SIZE];
  int written = snprintf(line, sizeof(line),
                         "%08lx,%lu,%lld,%s,%s,%s,%s,%s,%s,%d\n",
                         (unsigned long)s_boot_id, (unsigned long)sequence,
                         (long long)(esp_timer_get_time() / 1000), time, date,
                         latitude, longitude, altitude, speed, gps->valid);
  if (written >= 0 && written < sizeof(line))
    log_enqueue(LOG_TARGET_GPS, line);
  else
    counter_add(&s_queue_gps_lost);
}

void app_main(void) {
  s_boot_id = esp_random();
  s_log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(log_record_t));
  if (!s_log_queue) {
    ESP_LOGE(TAG, "Cannot create log queue");
    return;
  }
  if (xTaskCreate(logger_task, "sd_logger", LOGGER_TASK_STACK_SIZE, NULL,
                  LOGGER_TASK_PRIORITY, NULL) != pdPASS ||
      xTaskCreate(car_task, "car_sampler", 3072, NULL,
                  CAR_TASK_PRIORITY, NULL) != pdPASS) {
    ESP_LOGE(TAG, "Cannot create acquisition tasks");
    return;
  }

  nmea_parser_config_t config = NMEA_PARSER_CONFIG_DEFAULT();
  while (1) {
    nmea_parser_handle_t handle = nmea_parser_init(&config);
    if (handle) {
      esp_err_t err = nmea_parser_add_handler(handle, gps_event_handler, NULL);
      if (err == ESP_OK) {
        __atomic_store_n(&s_gps_handle, handle, __ATOMIC_RELEASE);
        ESP_LOGI(TAG, "GPS parser ready");
        return;
      }
      ESP_LOGE(TAG, "GPS handler registration failed: %s", esp_err_to_name(err));
      nmea_parser_deinit(handle);
    } else {
      ESP_LOGE(TAG, "GPS parser init failed; retrying");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
