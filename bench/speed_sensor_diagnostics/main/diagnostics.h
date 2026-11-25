#pragma once

#include "diag_logic.h"

#include "driver/gpio.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DIAG_PULL_DISABLED = 0,
  DIAG_PULL_DOWN,
  DIAG_PULL_UP,
} diag_pull_mode_t;

typedef struct {
  gpio_num_t input_gpio;
  gpio_num_t generator_gpio;
  uint32_t sample_interval_ms;
  uint32_t timeout_ms;
  uint32_t min_pulse_interval_us;
  uint32_t pulses_per_revolution;
  uint32_t generator_frequency_hz;
  uint32_t t4_rapid_edge_warning_us;
  uint32_t pcnt_glitch_filter_ns;
  diag_edge_mode_t edge_mode;
  diag_pull_mode_t pull_mode;
  bool rpm_relationship_confirmed;
  bool pcnt_enabled;
  bool t3_use_speed_sensor_driver;
} diag_config_t;

typedef struct {
  uint32_t boot_id;
  uint32_t run_id;
  int64_t boot_started_us;
  diag_config_t config;
  const char *init_status;
  esp_err_t init_error;
} diag_runtime_t;

typedef struct {
  int64_t read_time_us;
  uint64_t raw_count;
  uint64_t accepted_count;
  uint64_t rejected_count;
  int64_t last_event_us;
  int64_t since_last_event_us;
  int64_t filtered_last_event_us;
  int64_t since_last_filtered_event_us;
  int64_t raw_period_us;
  int64_t filtered_period_us;
  double raw_frequency_hz;
  double filtered_frequency_hz;
  diag_measurement_state_t raw_state;
  diag_measurement_state_t filtered_state;
  bool raw_period_valid;
  bool filtered_period_valid;
  bool counter_overflow;
} diag_capture_snapshot_t;

typedef esp_err_t (*diag_test_start_fn)(diag_runtime_t *runtime);
typedef esp_err_t (*diag_test_update_fn)(diag_runtime_t *runtime,
                                         int64_t now_us);
typedef esp_err_t (*diag_test_stop_fn)(diag_runtime_t *runtime);

typedef struct {
  const char *id;
  const char *name;
  diag_test_start_fn start;
  diag_test_update_fn update;
  diag_test_stop_fn stop;
} diag_test_t;

diag_config_t diag_config_from_menuconfig(void);
esp_err_t diag_capture_start(const diag_config_t *config,
                             diag_edge_mode_t edge,
                             diag_pull_mode_t pull,
                             uint32_t min_interval_us);
esp_err_t diag_capture_stop(void);
esp_err_t diag_capture_clear(void);
esp_err_t diag_capture_set_filter(uint32_t min_interval_us);
esp_err_t diag_capture_read(diag_capture_snapshot_t *snapshot);
esp_err_t diag_capture_read_level(int *level);
bool diag_capture_is_active(void);
const char *diag_edge_name(diag_edge_mode_t edge);
const char *diag_pull_name(diag_pull_mode_t pull);
const char *diag_state_name(diag_measurement_state_t state);
gpio_int_type_t diag_gpio_intr_type(diag_edge_mode_t edge);
void diag_print_boot_report(uint32_t boot_id);
void diag_print_test_header(const diag_runtime_t *runtime,
                            const diag_test_t *test,
                            const char *init_state,
                            const char *mode_config);
void diag_print_prefix(const diag_runtime_t *runtime, const char *test_id,
                       const char *mode_config, int64_t now_us);
int64_t diag_uptime_us(const diag_runtime_t *runtime, int64_t now_us);
esp_err_t diag_validate_input_gpio(gpio_num_t gpio);
esp_err_t diag_validate_input_gpio_config(gpio_num_t gpio,
                                          diag_pull_mode_t pull);
esp_err_t diag_validate_output_gpio(gpio_num_t gpio);
const char *diag_gpio_input_error(gpio_num_t gpio, diag_pull_mode_t pull);
const char *diag_gpio_output_error(gpio_num_t gpio);

esp_err_t test_t0_start(diag_runtime_t *runtime);
esp_err_t test_t0_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t0_stop(diag_runtime_t *runtime);

esp_err_t test_t1_start(diag_runtime_t *runtime);
esp_err_t test_t1_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t1_stop(diag_runtime_t *runtime);

esp_err_t test_t2_start(diag_runtime_t *runtime);
esp_err_t test_t2_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t2_stop(diag_runtime_t *runtime);

esp_err_t test_t3_start(diag_runtime_t *runtime);
esp_err_t test_t3_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t3_stop(diag_runtime_t *runtime);

esp_err_t test_t4_start(diag_runtime_t *runtime);
esp_err_t test_t4_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t4_stop(diag_runtime_t *runtime);
esp_err_t test_t4_mark(diag_runtime_t *runtime, const char *label);

esp_err_t test_t5_start(diag_runtime_t *runtime);
esp_err_t test_t5_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t5_stop(diag_runtime_t *runtime);
esp_err_t test_t5_note(diag_runtime_t *runtime, const char *note);

esp_err_t test_t6_start(diag_runtime_t *runtime);
esp_err_t test_t6_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t6_stop(diag_runtime_t *runtime);

esp_err_t test_t7_start(diag_runtime_t *runtime);
esp_err_t test_t7_update(diag_runtime_t *runtime, int64_t now_us);
esp_err_t test_t7_stop(diag_runtime_t *runtime);
esp_err_t test_t7_set_filter(diag_runtime_t *runtime, uint32_t min_interval_us);
