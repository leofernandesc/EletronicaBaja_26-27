#include "diagnostics.h"

#include <inttypes.h>
#include <stdio.h>

static int64_t s_last_heartbeat_us;

esp_err_t test_t0_start(diag_runtime_t *runtime) {
  (void)runtime;
  s_last_heartbeat_us = 0;
  return ESP_OK;
}

esp_err_t test_t0_update(diag_runtime_t *runtime, int64_t now_us) {
  if (s_last_heartbeat_us && now_us - s_last_heartbeat_us < 1000000)
    return ESP_OK;
  s_last_heartbeat_us = now_us;
  diag_print_prefix(runtime, "T0", "NO_SENSOR_IO", now_us);
  printf("status=RUNNING heartbeat=OK\r\n");
  return ESP_OK;
}

esp_err_t test_t0_stop(diag_runtime_t *runtime) {
  (void)runtime;
  return ESP_OK;
}
