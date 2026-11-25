#include "speed_measurement.h"
#include <math.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

bool IRAM_ATTR speed_measurement_pulse(speed_measurement_t *state,
                                      int64_t now_us, int64_t stop_timeout_us,
                                      uint32_t min_pulse_interval_us) {
  if (state->pulse_count) {
    int64_t interval_us = now_us - state->last_pulse_us;
    if (interval_us <= 0 || interval_us < min_pulse_interval_us) return false;
    // A new pulse after stopping begins a new period measurement.
    state->period_us = interval_us < stop_timeout_us ? interval_us : 0;
  }
  state->last_pulse_us = now_us;
  state->pulse_count++;
  return true;
}

bool speed_measurement_read(const speed_measurement_t *state, int64_t now_us,
                            int64_t stop_timeout_us,
                            float distance_per_pulse_m, float *speed_mps) {
  *speed_mps = 0.0f;
  if (!isfinite(distance_per_pulse_m) || distance_per_pulse_m <= 0.0f)
    return false;
  int64_t reference_us = state->pulse_count ? state->last_pulse_us
                                          : state->started_us;
  if (now_us - reference_us >= stop_timeout_us) return true;
  if (state->period_us <= 0) return false;
  *speed_mps = distance_per_pulse_m * (1000000.0f / (float)state->period_us);
  if (!isfinite(*speed_mps)) {
    *speed_mps = 0.0f;
    return false;
  }
  return true;
}
