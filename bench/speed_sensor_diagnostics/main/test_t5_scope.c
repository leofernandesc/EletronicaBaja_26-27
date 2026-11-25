#include "diagnostics.h"

#include "esp_timer.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static int64_t s_last_update_us;

esp_err_t test_t5_start(diag_runtime_t *runtime) {
  (void)runtime;
  s_last_update_us = 0;
  return ESP_OK;
}

esp_err_t test_t5_update(diag_runtime_t *runtime, int64_t now_us) {
  if (s_last_update_us && now_us - s_last_update_us < 1000000)
    return ESP_OK;
  s_last_update_us = now_us;
  diag_print_prefix(runtime, "T5", "SCOPE_ONLY_NO_GPIO_CAPTURE", now_us);
  printf("status=OBSERVE_EXTERNAL_SCOPE record=HIGH_V LOW_V PERIOD_US"
         " PULSE_WIDTH_US FREQUENCY_HZ NOISE MULTI_TRANSITIONS GAP_REFERENCE\r\n");
  return ESP_OK;
}

esp_err_t test_t5_note(diag_runtime_t *runtime, const char *note) {
  if (!note || !*note) return ESP_ERR_INVALID_ARG;
  char clean[81];
  size_t out = 0;
  for (size_t i = 0; note[i] && out < sizeof(clean) - 1; ++i) {
    unsigned char c = (unsigned char)note[i];
    if (isprint(c) && c != '\\' && c != '"') clean[out++] = (char)c;
  }
  clean[out] = '\0';
  diag_print_prefix(runtime, "T5", "SCOPE_ONLY", esp_timer_get_time());
  printf("MANUAL_SCOPE_NOTE=\"%s\"\r\n", clean);
  return ESP_OK;
}

esp_err_t test_t5_stop(diag_runtime_t *runtime) {
  (void)runtime;
  s_last_update_us = 0;
  return ESP_OK;
}
