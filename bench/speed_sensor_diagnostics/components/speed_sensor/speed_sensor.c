#include "speed_sensor.h"
#include "speed_measurement.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>

struct speed_sensor {
  speed_sensor_config_t config;
  int64_t stop_timeout_us;
  portMUX_TYPE lock;
  speed_measurement_t measurement;
};

static portMUX_TYPE s_mutex_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_lifecycle_mutex;
static bool s_gpio_owned[GPIO_NUM_MAX];
static bool s_service_ready;
static int s_service_core;

static SemaphoreHandle_t lifecycle_mutex(void) {
  portENTER_CRITICAL(&s_mutex_lock);
  SemaphoreHandle_t mutex = s_lifecycle_mutex;
  portEXIT_CRITICAL(&s_mutex_lock);
  if (mutex) return mutex;

  SemaphoreHandle_t candidate = xSemaphoreCreateMutex();
  if (!candidate) return NULL;
  portENTER_CRITICAL(&s_mutex_lock);
  if (!s_lifecycle_mutex) {
    s_lifecycle_mutex = candidate;
    candidate = NULL;
  }
  mutex = s_lifecycle_mutex;
  portEXIT_CRITICAL(&s_mutex_lock);
  if (candidate) vSemaphoreDelete(candidate);
  return mutex;
}

static void IRAM_ATTR pulse_isr(void *arg) {
  struct speed_sensor *sensor = arg;
  portENTER_CRITICAL_ISR(&sensor->lock);
  speed_measurement_pulse(&sensor->measurement, esp_timer_get_time(),
                           sensor->stop_timeout_us,
                           sensor->config.min_pulse_interval_us);
  portEXIT_CRITICAL_ISR(&sensor->lock);
}

esp_err_t speed_sensor_init(const speed_sensor_config_t *config,
                            speed_sensor_handle_t *out_handle) {
  if (!out_handle) return ESP_ERR_INVALID_ARG;
  *out_handle = NULL;
  if (!config || !GPIO_IS_VALID_GPIO(config->gpio) ||
      !isfinite(config->distance_per_pulse_m) ||
      config->distance_per_pulse_m < 0.0f || config->stop_timeout_ms == 0 ||
      (config->pulse_edge != GPIO_INTR_POSEDGE &&
       config->pulse_edge != GPIO_INTR_NEGEDGE) ||
      config->min_pulse_interval_us >= (int64_t)config->stop_timeout_ms * 1000)
    return ESP_ERR_INVALID_ARG;
#if CONFIG_IDF_TARGET_ESP32
  // These GPIOs are connected to the flash on typical classic ESP32 modules.
  if (config->gpio >= GPIO_NUM_6 && config->gpio <= GPIO_NUM_11)
    return ESP_ERR_INVALID_ARG;
#endif
#if !CONFIG_ESP_TIMER_IN_IRAM
  // This implementation requires an IRAM-safe timestamp source.
  return ESP_ERR_NOT_SUPPORTED;
#endif

  SemaphoreHandle_t mutex = lifecycle_mutex();
  if (!mutex) return ESP_ERR_NO_MEM;
  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t err = ESP_OK;
  struct speed_sensor *sensor = NULL;
  bool configured = false;
  bool handler_added = false;
  // A pinned lifecycle task cannot migrate during setup/removal. Removing on
  // the dispatch core guarantees its ISR has finished before freeing state.
  if (xTaskGetCoreID(NULL) != xPortGetCoreID() ||
      (s_service_ready && s_service_core != xPortGetCoreID()) ||
      s_gpio_owned[config->gpio]) {
    err = ESP_ERR_INVALID_STATE;
    goto done;
  }

  sensor = heap_caps_calloc(1, sizeof(*sensor),
                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!sensor) {
    err = ESP_ERR_NO_MEM;
    goto done;
  }
  sensor->config = *config;
  sensor->stop_timeout_us = (int64_t)config->stop_timeout_ms * 1000;
  sensor->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  if (!s_service_ready) {
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK) goto done;
    s_service_core = xPortGetCoreID();
    s_service_ready = true;
  }
  gpio_config_t pin_config = {
      .pin_bit_mask = 1ULL << config->gpio,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  err = gpio_config(&pin_config);
  if (err != ESP_OK) goto done;
  configured = true;
  sensor->measurement.started_us = esp_timer_get_time();
  err = gpio_isr_handler_add(config->gpio, pulse_isr, sensor);
  if (err != ESP_OK) goto done;
  handler_added = true;
  err = gpio_set_intr_type(config->gpio, config->pulse_edge);
  if (err != ESP_OK) goto done;
  s_gpio_owned[config->gpio] = true;
  *out_handle = sensor;

done:
  if (err != ESP_OK) {
    if (handler_added) gpio_isr_handler_remove(config->gpio);
    if (configured) gpio_reset_pin(config->gpio);
    free(sensor);
  }
  xSemaphoreGive(mutex);
  return err;
}

esp_err_t speed_sensor_read(speed_sensor_handle_t handle,
                            speed_sensor_reading_t *out_reading) {
  if (!handle || !out_reading) return ESP_ERR_INVALID_ARG;
  portENTER_CRITICAL(&handle->lock);
  speed_measurement_t snapshot = handle->measurement;
  int64_t now_us = esp_timer_get_time();
  portEXIT_CRITICAL(&handle->lock);
  *out_reading = (speed_sensor_reading_t){
      .pulse_count = snapshot.pulse_count,
      .last_pulse_us = snapshot.last_pulse_us,
      .period_us = snapshot.period_us,
  };
  out_reading->available = speed_measurement_read(
      &snapshot, now_us, handle->stop_timeout_us,
      handle->config.distance_per_pulse_m, &out_reading->speed_mps);
  return ESP_OK;
}

esp_err_t speed_sensor_deinit(speed_sensor_handle_t handle) {
  if (!handle) return ESP_ERR_INVALID_ARG;
  SemaphoreHandle_t mutex = lifecycle_mutex();
  if (!mutex) return ESP_ERR_NO_MEM;
  xSemaphoreTake(mutex, portMAX_DELAY);
  esp_err_t err = ESP_ERR_INVALID_STATE;
  if (xTaskGetCoreID(NULL) == xPortGetCoreID() &&
      s_service_core == xPortGetCoreID()) {
    err = gpio_isr_handler_remove(handle->config.gpio);
    if (err == ESP_OK) {
      gpio_reset_pin(handle->config.gpio);
      s_gpio_owned[handle->config.gpio] = false;
      free(handle);
    }
  }
  xSemaphoreGive(mutex);
  return err;
}
