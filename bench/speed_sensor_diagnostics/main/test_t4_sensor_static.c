#include "diagnostics.h"

#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static uint64_t s_previous_count;
static int s_last_level;
static int64_t s_level_since_us;

esp_err_t test_t4_start(diag_runtime_t *runtime) {
  s_previous_count = 0;
  s_last_level = -1;
  s_level_since_us = 0;
  esp_err_t err = diag_capture_start(&runtime->config, DIAG_EDGE_BOTH,
                                    runtime->config.pull_mode, 0);
  if (err != ESP_OK) return err;
  err = diag_capture_read_level(&s_last_level);
  if (err != ESP_OK) return err;
  s_level_since_us = esp_timer_get_time();
  return ESP_OK;
}

esp_err_t test_t4_update(diag_runtime_t *runtime, int64_t now_us) {
  (void)now_us;
  diag_capture_snapshot_t snapshot;
  int level = -1;
  esp_err_t err = diag_capture_read(&snapshot);
  if (err != ESP_OK) return err;
  now_us = snapshot.read_time_us;
  err = diag_capture_read_level(&level);
  if (err != ESP_OK) return err;
  /* T4 reports a sampled level duration, so timestamp it after reading level. */
  now_us = esp_timer_get_time();
  bool sampled_change = level != s_last_level;
  if (sampled_change) {
    s_last_level = level;
    s_level_since_us = now_us;
  }
  uint64_t delta = snapshot.raw_count - s_previous_count;
  s_previous_count = snapshot.raw_count;
  bool rapid = snapshot.raw_period_valid &&
               (uint64_t)snapshot.raw_period_us <
                   runtime->config.t4_rapid_edge_warning_us;
  diag_print_prefix(runtime, "T4", "SCOPE_MEASURES_VOLTAGE", now_us);
  printf("GPIO=%d LEVEL=%d LEVEL_AGE_SAMPLED_MS=%.3f"
         " LEVEL_DURATION=ESTIMATE_NOT_PULSE_WIDTH TRANSITIONS=%" PRIu64
         " DELTA=%" PRIu64 " LAST_US=",
         runtime->config.input_gpio, level,
         (double)(now_us - s_level_since_us) / 1000.0,
         snapshot.raw_count, delta);
  if (snapshot.raw_count) printf("%" PRId64, snapshot.last_event_us);
  else printf("--");
  printf(" LAST_TRANSITION_AGE_MS=");
  if (snapshot.raw_count)
    printf("%.3f", (double)snapshot.since_last_event_us / 1000.0);
  else
    printf("--");
  printf(" INTERVAL_US=");
  if (snapshot.raw_period_valid)
    printf("%" PRId64, snapshot.raw_period_us);
  else
    printf("--");
  printf(" SAMPLE_CHANGE=%d WARNING=%s RAW_STATE=%s\r\n", sampled_change,
         rapid ? "RAPID_TRANSITIONS_CHECK_SCOPE" : "NONE",
         diag_state_name(snapshot.raw_state));
  return ESP_OK;
}

esp_err_t test_t4_mark(diag_runtime_t *runtime, const char *label) {
  if (!label || (strcmp(label, "METAL") && strcmp(label, "GAP")))
    return ESP_ERR_INVALID_ARG;
  diag_print_prefix(runtime, "T4", "OPERATOR_MARKER", esp_timer_get_time());
  printf("MARK=%s note=operator_position_marker_not_sensor_validation\r\n",
         label);
  return ESP_OK;
}

esp_err_t test_t4_stop(diag_runtime_t *runtime) {
  (void)runtime;
  return diag_capture_stop();
}
