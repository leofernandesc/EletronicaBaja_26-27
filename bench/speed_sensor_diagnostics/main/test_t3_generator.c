#include "diagnostics.h"

#include "speed_sensor.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>

#define T3_LEDC_MODE LEDC_LOW_SPEED_MODE
#define T3_LEDC_TIMER LEDC_TIMER_0
#define T3_LEDC_CHANNEL LEDC_CHANNEL_0
#define T3_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define T3_DUTY_MAX (1U << 10)

typedef struct {
  int64_t read_time_us;
  uint64_t count;
  int64_t last_event_us;
  int64_t since_last_event_us;
  int64_t period_us;
  double frequency_hz;
  diag_measurement_state_t state;
  bool period_valid;
} t3_measurement_t;

static gpio_num_t s_output_gpio = GPIO_NUM_MAX;
static speed_sensor_handle_t s_sensor;
static bool s_driver_backend;
static bool s_ledc_timer_configured;
static bool s_ledc_channel_configured;
static bool s_ledc_output_active;
static uint32_t s_frequency_hz;
static uint64_t s_input_baseline;
static uint64_t s_previous_input_count;
static int64_t s_previous_window_us;
static int64_t s_capture_started_us;

static bool supported_frequency(uint32_t hz) {
  return hz == 1 || hz == 5 || hz == 10 || hz == 20 || hz == 100 ||
         hz == 500 || hz == 1000;
}

static esp_err_t generator_configure(gpio_num_t output_gpio, uint32_t hz) {
  if (!supported_frequency(hz) || diag_validate_output_gpio(output_gpio) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  ledc_timer_config_t timer = {
      .speed_mode = T3_LEDC_MODE,
      .duty_resolution = T3_DUTY_RESOLUTION,
      .timer_num = T3_LEDC_TIMER,
      .freq_hz = hz,
      .clk_cfg = hz <= 20 ? LEDC_USE_REF_TICK : LEDC_USE_APB_CLK,
      .deconfigure = false,
  };
  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK) return err;
  s_ledc_timer_configured = true;

  ledc_channel_config_t channel = {
      .gpio_num = output_gpio,
      .speed_mode = T3_LEDC_MODE,
      .channel = T3_LEDC_CHANNEL,
      .timer_sel = T3_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .deconfigure = false,
  };
  err = ledc_channel_config(&channel);
  if (err != ESP_OK) return err;
  s_ledc_channel_configured = true;
  s_output_gpio = output_gpio;
  s_frequency_hz = hz;
  return ESP_OK;
}

static esp_err_t generator_start(void) {
  if (!s_ledc_channel_configured) return ESP_ERR_INVALID_STATE;
  esp_err_t err = ledc_set_duty(T3_LEDC_MODE, T3_LEDC_CHANNEL,
                                T3_DUTY_MAX / 2);
  if (err != ESP_OK) return err;
  err = ledc_update_duty(T3_LEDC_MODE, T3_LEDC_CHANNEL);
  if (err == ESP_OK) s_ledc_output_active = true;
  return err;
}

static esp_err_t generator_cleanup(void) {
  esp_err_t result = ESP_OK;
  if (s_ledc_output_active) {
    esp_err_t err = ledc_stop(T3_LEDC_MODE, T3_LEDC_CHANNEL, 0);
    if (err == ESP_OK) s_ledc_output_active = false;
    else result = err;
  }
  if (s_ledc_channel_configured) {
    ledc_channel_config_t channel = {
        .speed_mode = T3_LEDC_MODE,
        .channel = T3_LEDC_CHANNEL,
        .deconfigure = true,
    };
    esp_err_t err = ledc_channel_config(&channel);
    if (err == ESP_OK) s_ledc_channel_configured = false;
    else if (result == ESP_OK) result = err;
  }
  if (s_ledc_timer_configured) {
    esp_err_t err = ledc_timer_pause(T3_LEDC_MODE, T3_LEDC_TIMER);
    if (err == ESP_OK) {
      ledc_timer_config_t timer = {
          .speed_mode = T3_LEDC_MODE,
          .timer_num = T3_LEDC_TIMER,
          .deconfigure = true,
      };
      err = ledc_timer_config(&timer);
      if (err == ESP_OK) s_ledc_timer_configured = false;
    }
    if (err != ESP_OK && result == ESP_OK) result = err;
  }
  if (s_output_gpio != GPIO_NUM_MAX) {
    esp_err_t err = gpio_reset_pin(s_output_gpio);
    if (err == ESP_OK) s_output_gpio = GPIO_NUM_MAX;
    else if (result == ESP_OK) result = err;
  }
  return result;
}

static void set_input_measurement_from_driver(const diag_runtime_t *runtime,
                                              t3_measurement_t *out) {
  if (!out) return;
  speed_sensor_reading_t reading = {0};
  out->state = DIAG_STATE_ERROR;
  if (speed_sensor_read(s_sensor, &reading) != ESP_OK) return;
  /* speed_sensor_read takes its own consistent state/time snapshot. Sample
   * after it returns so this external age can never precede that snapshot. */
  const int64_t read_time_us = esp_timer_get_time();
  out->count = reading.pulse_count;
  out->read_time_us = read_time_us;
  out->last_event_us = reading.last_pulse_us;
  const int64_t reference_us = reading.pulse_count > 0
                                   ? reading.last_pulse_us
                                   : s_capture_started_us;
  if (read_time_us < reference_us) return;
  out->since_last_event_us = read_time_us - reference_us;
  out->period_us = reading.period_us;
  if (out->since_last_event_us >=
      (int64_t)runtime->config.timeout_ms * 1000) {
    out->state = DIAG_STATE_TIMEOUT;
    return;
  }
  if (reading.pulse_count == 0) {
    out->state = DIAG_STATE_NO_EVENTS;
    return;
  }
  /* The original driver keeps pulse_count cumulative across a stop, but
   * clears period_us when the first event after timeout reanchors timing. */
  if (reading.pulse_count == 1 || reading.period_us <= 0) {
    out->state = DIAG_STATE_FIRST_EVENT;
    return;
  }
  if (reading.period_us > 0 &&
      reading.period_us < (int64_t)runtime->config.timeout_ms * 1000) {
    out->period_valid = true;
    out->frequency_hz = 1000000.0 / (double)reading.period_us;
    out->state = DIAG_STATE_MEASURING;
    return;
  }
  out->state = DIAG_STATE_WAITING_PERIOD;
}

static esp_err_t read_input_measurement(const diag_runtime_t *runtime,
                                       t3_measurement_t *out) {
  if (!out) return ESP_ERR_INVALID_ARG;
  *out = (t3_measurement_t){.state = DIAG_STATE_UNAVAILABLE};
  if (s_driver_backend) {
    set_input_measurement_from_driver(runtime, out);
    return out->state == DIAG_STATE_ERROR ? ESP_ERR_INVALID_STATE : ESP_OK;
  }
  diag_capture_snapshot_t snapshot;
  esp_err_t err = diag_capture_read(&snapshot);
  if (err != ESP_OK) return err;
  *out = (t3_measurement_t){
      .read_time_us = snapshot.read_time_us,
      .count = snapshot.raw_count,
      .last_event_us = snapshot.last_event_us,
      .since_last_event_us = snapshot.since_last_event_us,
      .period_us = snapshot.raw_period_us,
      .frequency_hz = snapshot.raw_frequency_hz,
      .state = snapshot.raw_state,
      .period_valid = snapshot.raw_period_valid,
  };
  return ESP_OK;
}

esp_err_t test_t3_start(diag_runtime_t *runtime) {
  gpio_num_t out = runtime->config.generator_gpio;
  if (!supported_frequency(runtime->config.generator_frequency_hz) ||
      out == runtime->config.input_gpio ||
      diag_validate_input_gpio_config(runtime->config.input_gpio,
                                      DIAG_PULL_DISABLED) != ESP_OK ||
      diag_validate_output_gpio(out) != ESP_OK)
    return ESP_ERR_INVALID_ARG;

  s_driver_backend = runtime->config.t3_use_speed_sensor_driver;
  s_sensor = NULL;
  s_output_gpio = GPIO_NUM_MAX;
  s_ledc_timer_configured = false;
  s_ledc_channel_configured = false;
  s_ledc_output_active = false;
  s_input_baseline = 0;
  s_previous_input_count = 0;

  esp_err_t err = generator_configure(out,
                                      runtime->config.generator_frequency_hz);
  if (err != ESP_OK) goto fail;

  if (s_driver_backend) {
    speed_sensor_config_t config = SPEED_SENSOR_CONFIG_DEFAULT();
    config.gpio = runtime->config.input_gpio;
    config.stop_timeout_ms = runtime->config.timeout_ms;
    config.min_pulse_interval_us = 0;
    config.distance_per_pulse_m = 0.0f;
    config.pulse_edge = GPIO_INTR_POSEDGE;
    err = speed_sensor_init(&config, &s_sensor);
  } else {
    err = diag_capture_start(&runtime->config, DIAG_EDGE_RISING,
                             DIAG_PULL_DISABLED, 0);
  }
  if (err != ESP_OK) goto fail;

  s_capture_started_us = esp_timer_get_time();
  t3_measurement_t baseline = {0};
  err = read_input_measurement(runtime, &baseline);
  if (err != ESP_OK) goto fail;
  s_input_baseline = baseline.count;
  s_previous_input_count = baseline.count;

  err = generator_start();
  if (err != ESP_OK) goto fail;
  s_previous_window_us = esp_timer_get_time();
  return ESP_OK;

fail:
  {
    esp_err_t cleanup_err = test_t3_stop(runtime);
    if (cleanup_err != ESP_OK) {
      diag_print_prefix(runtime, "T3", "STARTUP_CLEANUP", esp_timer_get_time());
      printf("STARTUP_CLEANUP_ERROR=%s code=%d\r\n",
             esp_err_to_name(cleanup_err), (int)cleanup_err);
      return cleanup_err;
    }
  }
  return err;
}

esp_err_t test_t3_update(diag_runtime_t *runtime, int64_t now_us) {
  (void)now_us;
  t3_measurement_t measurement;
  esp_err_t err = read_input_measurement(runtime, &measurement);
  if (err != ESP_OK) return err;
  now_us = measurement.read_time_us;
  int64_t elapsed_us = now_us - s_previous_window_us;
  if (elapsed_us <= 0) return ESP_ERR_INVALID_STATE;
  uint64_t input_delta = diag_counter_delta(measurement.count,
                                            &s_previous_input_count);
  uint64_t input_relative = 0;
  bool input_relative_valid = diag_counter_relative(measurement.count,
                                                    s_input_baseline,
                                                    &input_relative);
  double window_frequency_hz =
      (double)input_delta * 1000000.0 / (double)elapsed_us;
  double expected_window_events =
      (double)s_frequency_hz * (double)elapsed_us / 1000000.0;
  uint64_t expected_window_rounded =
      (uint64_t)(expected_window_events + 0.5);
  bool count_match = input_relative_valid &&
      diag_window_counts_match(expected_window_rounded, input_delta, 1);
  double period_error_percent = 0.0;
  bool period_error_valid = measurement.period_valid &&
      diag_frequency_error_percent((double)s_frequency_hz,
                                   measurement.frequency_hz,
                                   &period_error_percent);
  int level = -1;
  if (s_driver_backend) {
    level = gpio_get_level(runtime->config.input_gpio);
    err = level < 0 ? ESP_FAIL : ESP_OK;
  } else {
    err = diag_capture_read_level(&level);
  }
  if (err != ESP_OK) return err;

  diag_print_prefix(runtime, "T3", "EXTERNAL_JUMPER_1K_GPIO26_TO_GPIO27", now_us);
  printf("GPIO_IN=%d GPIO_GEN=%d EDGE=RISING BACKEND=%s GEN_HZ=%" PRIu32
         " EXPECTED_PERIOD_US=%" PRIu32 " PERIOD_MEASURED_US=",
         runtime->config.input_gpio, runtime->config.generator_gpio,
         s_driver_backend ? "SPEED_SENSOR_DRIVER" : "DIAGNOSTIC_ISR",
         s_frequency_hz, 1000000U / s_frequency_hz);
  if (measurement.period_valid)
    printf("%" PRId64, measurement.period_us);
  else
    printf("--");
  printf(" FREQ_PERIOD_HZ=");
  if (measurement.period_valid)
    printf("%.6f", measurement.frequency_hz);
  else
    printf("--");
  printf(" FREQ_WINDOW_HZ=%.6f ERROR_PERCENT=", window_frequency_hz);
  if (period_error_valid)
    printf("%.5f", period_error_percent);
  else
    printf("--");
  printf(" INPUT_BASELINE=%" PRIu64 " INPUT_TOTAL=%" PRIu64 " INPUT_REL=",
         s_input_baseline, measurement.count);
  if (input_relative_valid) printf("%" PRIu64, input_relative);
  else printf("COUNTER_RESET_OR_OVERFLOW");
  printf(" INPUT_DELTA=%" PRIu64 " EXPECTED_WINDOW_EVENTS=%.3f"
         " EXPECTED_ROUNDED=%" PRIu64 " COUNT_CHECK=%s LAST_EVENT_US=",
         input_delta, expected_window_events, expected_window_rounded,
         count_match ? "MATCH_WITH_1_EDGE_BOUNDARY_TOLERANCE"
                     : input_relative_valid ? "MISMATCH_REVIEW_SIGNAL_AND_CAPTURE"
                                            : "COUNTER_BASELINE_INVALID");
  if (measurement.count) printf("%" PRId64, measurement.last_event_us);
  else printf("--");
  printf(" AGE_MS=%.3f GPIO_LEVEL=%d RAW_STATE=%s\r\n",
         (double)measurement.since_last_event_us / 1000.0, level,
         diag_state_name(measurement.state));
  s_previous_window_us = now_us;
  return ESP_OK;
}

esp_err_t test_t3_stop(diag_runtime_t *runtime) {
  (void)runtime;
  esp_err_t result = generator_cleanup();
  if (s_sensor) {
    esp_err_t err = speed_sensor_deinit(s_sensor);
    if (err == ESP_OK) s_sensor = NULL;
    else if (result == ESP_OK) result = err;
  }
  if (!s_driver_backend) {
    esp_err_t err = diag_capture_stop();
    if (err != ESP_OK && result == ESP_OK) result = err;
  }
  return result;
}
