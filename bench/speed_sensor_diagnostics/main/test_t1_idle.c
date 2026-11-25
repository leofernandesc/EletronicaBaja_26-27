#include "diagnostics.h"

#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>

static uint64_t s_previous_count;
static uint64_t s_window_count;
static int64_t s_window_started_us;

esp_err_t test_t1_start(diag_runtime_t *runtime) {
  s_previous_count = 0;
  s_window_count = 0;
  s_window_started_us = esp_timer_get_time();
  return diag_capture_start(&runtime->config, DIAG_EDGE_RISING,
                            DIAG_PULL_DISABLED, 0);
}

esp_err_t test_t1_update(diag_runtime_t *runtime, int64_t now_us) {
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
  s_window_count += delta;
  int64_t elapsed = now_us - s_window_started_us;
  double irq_per_second = elapsed > 0
      ? (double)s_window_count * 1000000.0 / (double)elapsed : 0.0;
  if (elapsed >= 1000000) {
    s_window_count = 0;
    s_window_started_us = now_us;
  }
  const char *status = snapshot.raw_count == 0 ? "QUIET" : "UNEXPECTED_ACTIVITY";
  diag_print_prefix(runtime, "T1", "EXPECTED_EXTERNAL_GND_10K", now_us);
  printf("GPIO=%d LEVEL=%d IRQ_TOTAL=%" PRIu64 " IRQ_DELTA=%" PRIu64
         " IRQ_RATE_HZ=%.3f LAST_US=",
         runtime->config.input_gpio, level, snapshot.raw_count, delta,
         irq_per_second);
  if (snapshot.raw_count) printf("%" PRId64, snapshot.last_event_us);
  else printf("--");
  printf(" LAST_AGE_MS=");
  if (snapshot.raw_count)
    printf("%.3f", (double)snapshot.since_last_event_us / 1000.0);
  else
    printf("--");
  printf(" status=%s\r\n", status);
  return ESP_OK;
}

esp_err_t test_t1_stop(diag_runtime_t *runtime) {
  (void)runtime;
  return diag_capture_stop();
}
