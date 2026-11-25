#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct speed_sensor *speed_sensor_handle_t;

typedef struct {
  gpio_num_t gpio;
  float distance_per_pulse_m; // Zero means calibration is unavailable.
  uint32_t stop_timeout_ms;
  uint32_t min_pulse_interval_us; // Zero disables interval rejection.
  gpio_int_type_t pulse_edge; // GPIO_INTR_POSEDGE or GPIO_INTR_NEGEDGE.
} speed_sensor_config_t;

#define SPEED_SENSOR_CONFIG_DEFAULT()                                          \
  {.gpio = GPIO_NUM_27, .distance_per_pulse_m = 0.0f,                            \
   .stop_timeout_ms = 1000, .min_pulse_interval_us = 0,                          \
   .pulse_edge = GPIO_INTR_POSEDGE}

typedef struct {
  bool available;
  float speed_mps; // Only meaningful when available is true.
  uint64_t pulse_count; // Accepted pulses since initialization.
  int64_t last_pulse_us; // Monotonic time; consult pulse_count before using.
  int64_t period_us; // Last measured interval, retained on timeout; 0 if unknown.
} speed_sensor_reading_t;

/**
 * Initialize an independent sensor. No task or PCNT unit is allocated.
 * The component installs and retains the global GPIO ISR service in IRAM.
 * Initialize it before other users install that service; an existing foreign
 * service returns ESP_ERR_INVALID_STATE. Other users may then share it.
 * All init/deinit calls must execute in a task pinned to the core that
 * initialized the service (the default ESP-IDF main task meets this condition).
 * CONFIG_ESP_TIMER_IN_IRAM must be enabled; otherwise init returns
 * ESP_ERR_NOT_SUPPORTED.
 * GPIOs must be exclusively assigned to this component by the application.
 * Duplicate GPIOs within this component are rejected.
 */
esp_err_t speed_sensor_init(const speed_sensor_config_t *config,
                            speed_sensor_handle_t *out_handle);

/** Read a consistent snapshot without waiting for pulses or doing any I/O. */
esp_err_t speed_sensor_read(speed_sensor_handle_t handle,
                            speed_sensor_reading_t *out_reading);

/**
 * Remove this instance's handler and reset its GPIO. Stop all reads of this
 * handle before calling; cleanup must not run concurrently with reads.
 * The shared GPIO ISR service remains installed for other users/instances.
 */
esp_err_t speed_sensor_deinit(speed_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif
