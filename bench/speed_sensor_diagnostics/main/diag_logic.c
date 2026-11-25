#include "diag_logic.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define DIAG_IRAM_ATTR IRAM_ATTR
#else
#define DIAG_IRAM_ATTR
#endif

void diag_measurement_reset(diag_measurement_t *measurement,
                            int64_t now_us, int64_t timeout_us,
                            uint32_t min_interval_us) {
  if (!measurement) return;
  memset(measurement, 0, sizeof(*measurement));
  measurement->started_us = now_us;
  measurement->timeout_us = timeout_us;
  measurement->min_interval_us = min_interval_us;
  measurement->filtered.started_us = now_us;
}

bool diag_measurement_set_filter(diag_measurement_t *measurement,
                                 int64_t now_us, uint32_t min_interval_us) {
  if (!measurement || measurement->timeout_us <= 0 ||
      min_interval_us >= (uint64_t)measurement->timeout_us ||
      now_us < measurement->started_us ||
      (measurement->raw_count > 0 && now_us < measurement->last_raw_us))
    return false;
  if (measurement->filtered.pulse_count >
      UINT64_MAX - measurement->accepted_count_base) {
    measurement->accepted_count_base = UINT64_MAX;
    measurement->counter_overflow = true;
  } else {
    measurement->accepted_count_base += measurement->filtered.pulse_count;
  }
  measurement->filtered = (speed_measurement_t){.started_us = now_us};
  measurement->filtered_restarted = false;
  measurement->min_interval_us = min_interval_us;
  return true;
}

bool DIAG_IRAM_ATTR diag_measurement_record(diag_measurement_t *measurement,
                                            int64_t now_us) {
  if (!measurement || measurement->timeout_us <= 0 ||
      now_us < measurement->started_us) {
    return false;
  }

  bool raw_timestamp_valid = true;
  measurement->raw_restarted = false;
  if (measurement->raw_count == UINT64_MAX) {
    measurement->counter_overflow = true;
    raw_timestamp_valid = false;
  } else {
    if (measurement->raw_count > 0) {
      int64_t interval = now_us - measurement->last_raw_us;
      measurement->raw_period_us = interval > 0 ? interval : 0;
      measurement->raw_restarted = interval >= measurement->timeout_us;
      if (interval <= 0) raw_timestamp_valid = false;
    }
    measurement->raw_count++;
    measurement->last_raw_us = now_us;
  }

  if (measurement->filtered.pulse_count == UINT64_MAX)
    measurement->counter_overflow = true;
  bool filtered_had_previous = measurement->filtered.pulse_count > 0;
  int64_t filtered_interval = filtered_had_previous
      ? now_us - measurement->filtered.last_pulse_us : 0;
  bool accepted = raw_timestamp_valid &&
      measurement->filtered.pulse_count < UINT64_MAX &&
      speed_measurement_pulse(&measurement->filtered, now_us,
                              measurement->timeout_us,
                              measurement->min_interval_us);
  if (accepted) {
    measurement->filtered_restarted = filtered_had_previous &&
        filtered_interval >= measurement->timeout_us;
  }
  if (!accepted) {
    if (measurement->rejected_count < UINT64_MAX)
      measurement->rejected_count++;
    else
      measurement->counter_overflow = true;
  }
  return accepted;
}

bool diag_measurement_read(const diag_measurement_t *measurement,
                           int64_t now_us,
                           diag_measurement_snapshot_t *snapshot) {
  if (!measurement || !snapshot) return false;
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->raw_state = DIAG_STATE_ERROR;
  snapshot->filtered_state = DIAG_STATE_ERROR;
  if (measurement->timeout_us <= 0 || now_us < measurement->started_us ||
      now_us < measurement->filtered.started_us ||
      (measurement->raw_count > 0 && now_us < measurement->last_raw_us) ||
      (measurement->filtered.pulse_count > 0 &&
       now_us < measurement->filtered.last_pulse_us))
    return false;

  snapshot->raw_count = measurement->raw_count;
  if (measurement->filtered.pulse_count >
      UINT64_MAX - measurement->accepted_count_base) {
    snapshot->accepted_count = UINT64_MAX;
    snapshot->counter_overflow = true;
  } else {
    snapshot->accepted_count = measurement->accepted_count_base +
                               measurement->filtered.pulse_count;
  }
  snapshot->rejected_count = measurement->rejected_count;
  if (measurement->raw_count > 0) {
    snapshot->last_event_us = measurement->last_raw_us;
  }
  if (measurement->filtered.pulse_count > 0) {
    snapshot->filtered_last_event_us = measurement->filtered.last_pulse_us;
  }
  snapshot->counter_overflow =
      snapshot->counter_overflow || measurement->counter_overflow;

  const int64_t raw_reference_us = measurement->raw_count > 0
                                       ? measurement->last_raw_us
                                       : measurement->started_us;
  const int64_t filtered_reference_us =
      measurement->filtered.pulse_count > 0
          ? measurement->filtered.last_pulse_us
          : measurement->filtered.started_us;
  snapshot->since_last_event_us = now_us - raw_reference_us;
  snapshot->since_last_filtered_event_us = now_us - filtered_reference_us;

  if (snapshot->since_last_event_us >= measurement->timeout_us) {
    snapshot->raw_state = DIAG_STATE_TIMEOUT;
  } else if (measurement->raw_count == 0) {
    snapshot->raw_state = DIAG_STATE_NO_EVENTS;
  } else if (measurement->raw_count == 1 || measurement->raw_restarted) {
    snapshot->raw_state = DIAG_STATE_FIRST_EVENT;
  } else if (measurement->raw_period_us > 0 &&
             measurement->raw_period_us < measurement->timeout_us) {
    snapshot->raw_period_valid = true;
    snapshot->raw_period_us = measurement->raw_period_us;
    snapshot->raw_frequency_hz =
        1000000.0 / (double)measurement->raw_period_us;
    snapshot->raw_state = DIAG_STATE_MEASURING;
  } else {
    snapshot->raw_state = DIAG_STATE_WAITING_PERIOD;
  }

  if (snapshot->since_last_filtered_event_us >= measurement->timeout_us) {
    snapshot->filtered_state = DIAG_STATE_TIMEOUT;
  } else if (measurement->filtered.pulse_count == 0) {
    snapshot->filtered_state = DIAG_STATE_NO_EVENTS;
  } else if (measurement->filtered.pulse_count == 1 ||
             measurement->filtered_restarted) {
    snapshot->filtered_state = DIAG_STATE_FIRST_EVENT;
  } else if (measurement->filtered.period_us > 0 &&
             measurement->filtered.period_us < measurement->timeout_us) {
    snapshot->filtered_period_valid = true;
    snapshot->filtered_period_us = measurement->filtered.period_us;
    snapshot->filtered_frequency_hz =
        1000000.0 / (double)measurement->filtered.period_us;
    snapshot->filtered_state = DIAG_STATE_MEASURING;
  } else {
    snapshot->filtered_state = DIAG_STATE_WAITING_PERIOD;
  }
  return true;
}

bool diag_measurement_read_consistent(diag_measurement_copy_fn copy,
                                     diag_monotonic_clock_fn clock,
                                     void *context,
                                     diag_measurement_snapshot_t *snapshot) {
  if (!copy || !clock || !snapshot) return false;

  diag_measurement_t local = {0};
  if (!copy(context, &local)) return false;

  /* Event timestamps can be recorded by the ISR during the copy callback.
   * Take the reference only after the callback has returned its snapshot. */
  const int64_t read_time_us = clock(context);
  if (!diag_measurement_read(&local, read_time_us, snapshot)) return false;
  snapshot->read_time_us = read_time_us;
  return true;
}

bool diag_edge_accepts(diag_edge_mode_t selected, diag_event_edge_t event) {
  if (selected == DIAG_EDGE_BOTH) return true;
  if (selected == DIAG_EDGE_RISING) return event == DIAG_EVENT_RISING;
  if (selected == DIAG_EDGE_FALLING) return event == DIAG_EVENT_FALLING;
  return false;
}

bool diag_calculate_rpm(int64_t period_us, uint32_t pulses_per_revolution,
                        bool relationship_confirmed, double *frequency_hz,
                        double *rpm) {
  if (!frequency_hz || !rpm) return false;
  *frequency_hz = 0.0;
  *rpm = 0.0;
  if (!relationship_confirmed || period_us <= 0 ||
      pulses_per_revolution == 0)
    return false;
  double frequency = 1000000.0 / (double)period_us;
  double rotations_per_minute =
      (frequency * 60.0) / (double)pulses_per_revolution;
  if (!isfinite(frequency) || !isfinite(rotations_per_minute)) return false;
  *frequency_hz = frequency;
  *rpm = rotations_per_minute;
  return true;
}

bool diag_frequency_error_percent(double expected_hz, double measured_hz,
                                  double *error_percent) {
  if (!error_percent) return false;
  *error_percent = 0.0;
  if (!isfinite(expected_hz) || !isfinite(measured_hz) || expected_hz <= 0.0 ||
      measured_hz < 0.0)
    return false;
  *error_percent = fabs(measured_hz - expected_hz) / expected_hz * 100.0;
  return isfinite(*error_percent);
}

bool diag_window_counts_match(uint64_t expected, uint64_t observed,
                              uint64_t boundary_tolerance) {
  uint64_t difference = expected > observed ? expected - observed
                                             : observed - expected;
  return difference <= boundary_tolerance;
}

uint64_t diag_counter_delta(uint64_t current, uint64_t *previous) {
  if (!previous) return 0;
  uint64_t delta = current >= *previous ? current - *previous : 0;
  *previous = current;
  return delta;
}

bool diag_counter_relative(uint64_t total, uint64_t baseline,
                           uint64_t *relative) {
  if (!relative || total < baseline) return false;
  *relative = total - baseline;
  return true;
}

const char *diag_gpio_policy_error(int gpio, bool output) {
  if (gpio < 0 || gpio > 39) return "GPIO fora do intervalo do ESP32 classico";
  if (gpio >= 6 && gpio <= 11) return "GPIO reservado para a flash";
  if (gpio == 37 || gpio == 38)
    return "GPIO nao exposto no modulo ESP32-WROOM-32";
  if (gpio == 0 || gpio == 2 || gpio == 5 || gpio == 12 || gpio == 15)
    return "GPIO de strapping; pode interferir no boot";
  if (gpio == 1 || gpio == 3) return "GPIO reservado para UART0/monitor serial";
  if (output && gpio >= 34) return "GPIO 34-39 sao somente entrada";
  return NULL;
}
