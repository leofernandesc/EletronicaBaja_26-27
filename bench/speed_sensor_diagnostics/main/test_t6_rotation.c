#include "diagnostics.h"

#include "speed_sensor.h"

#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>

static speed_sensor_handle_t s_sensor;
static uint64_t s_previous_count;
static int64_t s_started_us;

esp_err_t test_t6_start(diag_runtime_t *runtime) {
  speed_sensor_config_t config = SPEED_SENSOR_CONFIG_DEFAULT();
  config.gpio = runtime->config.input_gpio;
  config.stop_timeout_ms = runtime->config.timeout_ms;
  config.min_pulse_interval_us = 0;
  config.distance_per_pulse_m = 0.0f;
  config.pulse_edge = runtime->config.edge_mode == DIAG_EDGE_FALLING
                          ? GPIO_INTR_NEGEDGE
                          : GPIO_INTR_POSEDGE;
  if (runtime->config.edge_mode == DIAG_EDGE_BOTH) return ESP_ERR_INVALID_ARG;
  s_sensor = NULL;
  s_previous_count = 0;
  s_started_us = esp_timer_get_time();
  return speed_sensor_init(&config, &s_sensor);
}

esp_err_t test_t6_update(diag_runtime_t *runtime, int64_t now_us) {
  (void)now_us;
  speed_sensor_reading_t reading;
  esp_err_t err = speed_sensor_read(s_sensor, &reading);
  if (err != ESP_OK) return err;
  /* The driver snapshots its ISR state internally. Read the age reference
   * afterward so a pulse concurrent with this call cannot appear in the
   * snapshot later than the caller's timestamp. */
  now_us = esp_timer_get_time();
  uint64_t delta = reading.pulse_count - s_previous_count;
  s_previous_count = reading.pulse_count;
  int level = gpio_get_level(runtime->config.input_gpio);
  const int64_t timeout_us = (int64_t)runtime->config.timeout_ms * 1000;
  const int64_t last_reference_us = reading.pulse_count > 0
                                        ? reading.last_pulse_us
                                        : s_started_us;
  if (now_us < last_reference_us) return ESP_ERR_INVALID_STATE;
  const int64_t age_us = now_us - last_reference_us;
  bool period_available = reading.pulse_count >= 2 && reading.period_us > 0 &&
      age_us < timeout_us;
  bool timed_out = age_us >= timeout_us;
  diag_print_prefix(runtime, "T6", "RPM_DISABLED_PULSE_PER_REV_UNVERIFIED", now_us);
  printf("GPIO=%d EDGE=%s PULL=DISABLED LEVEL=%d IRQ_TOTAL=%" PRIu64 " DELTA=%" PRIu64
         " LAST_PULSE_US=",
         runtime->config.input_gpio, diag_edge_name(runtime->config.edge_mode),
         level, reading.pulse_count, delta);
  if (reading.pulse_count) printf("%" PRId64, reading.last_pulse_us);
  else printf("--");
  printf(" PERIOD_US=");
  if (period_available)
    printf("%" PRId64, reading.period_us);
  else
    printf("--");
  printf(" FREQ_EVENT_HZ=");
  if (period_available)
    printf("%.6f", 1000000.0 / (double)reading.period_us);
  else
    printf("%s", timed_out ? "0.000000" : "--");
  printf(" AGE_MS=");
  if (reading.pulse_count)
    printf("%.3f", (double)age_us / 1000.0);
  else
    printf("--");
  printf(" STATE=%s\r\n", timed_out ? "TIMEOUT"
                                    : reading.pulse_count < 2 ||
                                              !period_available
                                          ? (reading.pulse_count ? "PRIMEIRO_EVENTO"
                                                                  : "SEM_PULSOS")
                                          : "MEDINDO");
  return ESP_OK;
}

esp_err_t test_t6_stop(diag_runtime_t *runtime) {
  (void)runtime;
  if (!s_sensor) return ESP_OK;
  esp_err_t err = speed_sensor_deinit(s_sensor);
  if (err == ESP_OK) s_sensor = NULL;
  return err;
}
