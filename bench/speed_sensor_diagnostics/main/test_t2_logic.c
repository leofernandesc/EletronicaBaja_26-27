#include "diagnostics.h"

#include <inttypes.h>
#include <stdio.h>

static uint64_t s_previous_count;

esp_err_t test_t2_start(diag_runtime_t *runtime) {
  s_previous_count = 0;
  return diag_capture_start(&runtime->config, runtime->config.edge_mode,
                            runtime->config.pull_mode, 0);
}

esp_err_t test_t2_update(diag_runtime_t *runtime, int64_t now_us) {
  (void)now_us;
  diag_capture_snapshot_t snapshot;
  int level = -1;
  esp_err_t err = diag_capture_read(&snapshot);
  if (err != ESP_OK) return err;
  now_us = snapshot.read_time_us;
  err = diag_capture_read_level(&level);
  if (err != ESP_OK) return err;
  uint64_t delta = snapshot.raw_count - s_previous_count;
  s_previous_count = snapshot.raw_count;
  diag_print_prefix(runtime, "T2", "RPM_DISABLED", now_us);
  printf("GPIO=%d EDGE=%s LEVEL=%d IRQ_TOTAL=%" PRIu64
         " DELTA=%" PRIu64 " LAST_US=",
         runtime->config.input_gpio, diag_edge_name(runtime->config.edge_mode),
         level, snapshot.raw_count, delta);
  if (snapshot.raw_count) printf("%" PRId64, snapshot.last_event_us);
  else printf("--");
  printf(" PERIOD_US=");
  if (snapshot.raw_period_valid)
    printf("%" PRId64, snapshot.raw_period_us);
  else
    printf("--");
  printf(" GAP_US=");
  if (snapshot.raw_count)
    printf("%.0f", (double)snapshot.since_last_event_us);
  else
    printf("--");
  printf(" RAW_STATE=%s STATUS=NO_RPM\r\n",
         diag_state_name(snapshot.raw_state));
  return ESP_OK;
}

esp_err_t test_t2_stop(diag_runtime_t *runtime) {
  (void)runtime;
  return diag_capture_stop();
}
