#pragma once

// Internal pulse model, also built directly for deterministic host tests.
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  int64_t started_us;
  int64_t last_pulse_us;
  int64_t period_us;
  uint64_t pulse_count;
} speed_measurement_t;

bool speed_measurement_pulse(speed_measurement_t *state, int64_t now_us,
                             int64_t stop_timeout_us,
                             uint32_t min_pulse_interval_us);
bool speed_measurement_read(const speed_measurement_t *state, int64_t now_us,
                            int64_t stop_timeout_us,
                            float distance_per_pulse_m, float *speed_mps);
