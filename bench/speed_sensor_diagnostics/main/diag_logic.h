#pragma once

#include "speed_measurement.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DIAG_EDGE_RISING = 0,
  DIAG_EDGE_FALLING,
  DIAG_EDGE_BOTH,
} diag_edge_mode_t;

typedef enum {
  DIAG_EVENT_RISING = 0,
  DIAG_EVENT_FALLING,
} diag_event_edge_t;

typedef enum {
  DIAG_STATE_UNAVAILABLE = 0,
  DIAG_STATE_NO_EVENTS,
  DIAG_STATE_FIRST_EVENT,
  DIAG_STATE_WAITING_PERIOD,
  DIAG_STATE_MEASURING,
  DIAG_STATE_TIMEOUT,
  DIAG_STATE_ERROR,
} diag_measurement_state_t;

typedef struct {
  uint64_t raw_count;
  uint64_t rejected_count;
  uint64_t accepted_count_base;
  int64_t started_us;
  int64_t last_raw_us;
  int64_t raw_period_us;
  int64_t timeout_us;
  uint32_t min_interval_us;
  speed_measurement_t filtered;
  bool counter_overflow;
  bool raw_restarted;
  bool filtered_restarted;
} diag_measurement_t;

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
} diag_measurement_snapshot_t;

typedef bool (*diag_measurement_copy_fn)(void *context,
                                         diag_measurement_t *measurement);
typedef int64_t (*diag_monotonic_clock_fn)(void *context);

void diag_measurement_reset(diag_measurement_t *measurement,
                            int64_t now_us, int64_t timeout_us,
                            uint32_t min_interval_us);
bool diag_measurement_set_filter(diag_measurement_t *measurement,
                                 int64_t now_us, uint32_t min_interval_us);
bool diag_measurement_record(diag_measurement_t *measurement, int64_t now_us);
bool diag_measurement_read(const diag_measurement_t *measurement,
                           int64_t now_us,
                           diag_measurement_snapshot_t *snapshot);
/* Copies the event state first, then samples the same monotonic clock used by
 * event timestamps. Firmware capture backends use this to prevent a read time
 * older than an event already included in the copied snapshot. */
bool diag_measurement_read_consistent(diag_measurement_copy_fn copy,
                                     diag_monotonic_clock_fn clock,
                                     void *context,
                                     diag_measurement_snapshot_t *snapshot);
bool diag_edge_accepts(diag_edge_mode_t selected, diag_event_edge_t event);
bool diag_calculate_rpm(int64_t period_us, uint32_t pulses_per_revolution,
                        bool relationship_confirmed, double *frequency_hz,
                        double *rpm);
bool diag_frequency_error_percent(double expected_hz, double measured_hz,
                                  double *error_percent);
bool diag_window_counts_match(uint64_t expected, uint64_t observed,
                              uint64_t boundary_tolerance);
uint64_t diag_counter_delta(uint64_t current, uint64_t *previous);
bool diag_counter_relative(uint64_t total, uint64_t baseline,
                           uint64_t *relative);
const char *diag_gpio_policy_error(int gpio, bool output);
