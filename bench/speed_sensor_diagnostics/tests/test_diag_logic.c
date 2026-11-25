#include "diag_logic.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_edge_selection(void) {
  const diag_event_edge_t sequence[] = {
      DIAG_EVENT_RISING, DIAG_EVENT_FALLING,
  };
  unsigned rising = 0, falling = 0, both = 0;
  for (unsigned i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i) {
    rising += diag_edge_accepts(DIAG_EDGE_RISING, sequence[i]);
    falling += diag_edge_accepts(DIAG_EDGE_FALLING, sequence[i]);
    both += diag_edge_accepts(DIAG_EDGE_BOTH, sequence[i]);
  }
  assert(rising == 1 && falling == 1 && both == 2);
}

static void test_period_frequency_rpm(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 100, 5000000, 0);
  assert(diag_measurement_record(&measurement, 100));
  assert(diag_measurement_record(&measurement, 600100));
  assert(diag_measurement_read(&measurement, 600100, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.filtered_state == DIAG_STATE_MEASURING);
  assert(snapshot.last_event_us == 600100);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 600000);
  assert(fabs(snapshot.raw_frequency_hz - (1000000.0 / 600000.0)) < 1e-12);
  double frequency = 0.0, rpm = 0.0;
  assert(diag_calculate_rpm(600000, 1, true, &frequency, &rpm));
  assert(fabs(frequency - (5.0 / 3.0)) < 1e-12);
  assert(fabs(rpm - 100.0) < 1e-12);
  assert(!diag_calculate_rpm(600000, 1, false, &frequency, &rpm));
  assert(frequency == 0.0 && rpm == 0.0);
  assert(!diag_calculate_rpm(0, 1, true, &frequency, &rpm));
  assert(!diag_calculate_rpm(1, 0, true, &frequency, &rpm));
}

static void test_first_timeout_resume_and_no_history(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 0, 5000000, 0);
  assert(diag_measurement_read(&measurement, 0, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_NO_EVENTS);
  assert(snapshot.filtered_state == DIAG_STATE_NO_EVENTS);
  assert(diag_measurement_record(&measurement, 100));
  assert(diag_measurement_read(&measurement, 100, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(!snapshot.raw_period_valid && !snapshot.filtered_period_valid);
  assert(diag_measurement_record(&measurement, 1000100));
  assert(diag_measurement_read(&measurement, 1000100, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.filtered_state == DIAG_STATE_MEASURING);
  assert(snapshot.raw_period_us == 1000000);
  assert(diag_measurement_read(&measurement, 6000100, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_TIMEOUT);
  assert(snapshot.filtered_state == DIAG_STATE_TIMEOUT);
  assert(!snapshot.raw_period_valid && !snapshot.filtered_period_valid);
  assert(snapshot.raw_frequency_hz == 0.0 && snapshot.filtered_frequency_hz == 0.0);
  assert(diag_measurement_record(&measurement, 7000000));
  assert(diag_measurement_read(&measurement, 7000000, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(!snapshot.raw_period_valid);
  assert(diag_measurement_record(&measurement, 8000000));
  assert(diag_measurement_read(&measurement, 8000000, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.filtered_state == DIAG_STATE_MEASURING);
  assert(snapshot.raw_period_us == 1000000);
  assert(snapshot.raw_count == 4);
}

static void test_filter_and_raw_count(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 0, 5000000, 1000);
  assert(diag_measurement_record(&measurement, 100));
  assert(!diag_measurement_record(&measurement, 200));
  assert(diag_measurement_record(&measurement, 1200));
  assert(diag_measurement_record(&measurement, 2300));
  assert(diag_measurement_read(&measurement, 2300, &snapshot));
  assert(snapshot.raw_count == 4);
  assert(snapshot.accepted_count == 3);
  assert(snapshot.rejected_count == 1);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 1100);
  assert(snapshot.filtered_period_valid && snapshot.filtered_period_us == 1100);
}

static void test_synthetic_rates_and_window_edges(void) {
  const unsigned rates[] = {1, 5, 10, 20, 100, 500, 1000};
  for (unsigned n = 0; n < sizeof(rates) / sizeof(rates[0]); ++n) {
    const int64_t period = 1000000 / rates[n];
    diag_measurement_t measurement;
    diag_measurement_snapshot_t snapshot;
    diag_measurement_reset(&measurement, 0, 5000000, 0);
    for (unsigned i = 0; i < 20; ++i)
      assert(diag_measurement_record(&measurement, (int64_t)i * period));
    assert(diag_measurement_read(&measurement, 19LL * period, &snapshot));
    assert(snapshot.raw_period_valid);
    assert(fabs(snapshot.raw_frequency_hz - rates[n]) < 1e-12);
    double error = 0.0;
    assert(diag_frequency_error_percent(rates[n], snapshot.raw_frequency_hz,
                                        &error));
    assert(error < 1e-12);
  }
  assert(diag_window_counts_match(100, 99, 1));
  assert(diag_window_counts_match(100, 101, 1));
  assert(!diag_window_counts_match(100, 98, 1));
  assert(!diag_frequency_error_percent(0.0, 1.0, &(double){0}));
}

static void test_restart_resets_state(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 10, 5000000, 0);
  assert(diag_measurement_record(&measurement, 10));
  assert(diag_measurement_record(&measurement, 2010));
  assert(measurement.raw_count == 2);
  diag_measurement_reset(&measurement, 5000, 5000000, 200);
  assert(diag_measurement_read(&measurement, 5000, &snapshot));
  assert(snapshot.raw_count == 0 && snapshot.accepted_count == 0 &&
         snapshot.rejected_count == 0 &&
         snapshot.raw_state == DIAG_STATE_NO_EVENTS &&
         snapshot.filtered_state == DIAG_STATE_NO_EVENTS);
  assert(measurement.min_interval_us == 200);
}

static void test_filter_change_preserves_filtered_total(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 0, 5000000, 0);
  assert(diag_measurement_record(&measurement, 100));
  assert(diag_measurement_record(&measurement, 1100));
  assert(diag_measurement_read(&measurement, 1100, &snapshot));
  assert(snapshot.accepted_count == 2 && snapshot.raw_count == 2);
  assert(diag_measurement_set_filter(&measurement, 1200, 500));
  assert(diag_measurement_read(&measurement, 1200, &snapshot));
  uint64_t previous_filtered = snapshot.accepted_count;
  assert(diag_measurement_read(&measurement, 1300, &snapshot));
  assert(diag_counter_delta(snapshot.accepted_count, &previous_filtered) == 0);
  assert(snapshot.accepted_count == 2);
  assert(diag_measurement_record(&measurement, 1400));
  assert(diag_measurement_read(&measurement, 1400, &snapshot));
  assert(snapshot.accepted_count == 3 && snapshot.raw_count == 3);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(!snapshot.filtered_period_valid);
  assert(diag_measurement_record(&measurement, 1900));
  assert(diag_measurement_read(&measurement, 1900, &snapshot));
  assert(snapshot.accepted_count == 4 && snapshot.raw_count == 4);
  assert(snapshot.filtered_period_valid && snapshot.filtered_period_us == 500);
}

static void test_independent_raw_and_filtered_states(void) {
  diag_measurement_t measurement;
  diag_measurement_snapshot_t snapshot;
  diag_measurement_reset(&measurement, 0, 20000, 5000);
  assert(diag_measurement_record(&measurement, 100));
  for (int64_t i = 1; i <= 3000; ++i)
    assert(!diag_measurement_record(&measurement, 100 + i));
  assert(diag_measurement_read(&measurement, 3100, &snapshot));
  assert(snapshot.raw_count == 3001);
  assert(snapshot.accepted_count == 1);
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 1);
  assert(!snapshot.filtered_period_valid);
  assert(snapshot.filtered_frequency_hz == 0.0);
}

static void test_counter_baselines_and_gpio_policy(void) {
  uint64_t previous = 900;
  assert(diag_counter_delta(900, &previous) == 0);
  assert(diag_counter_delta(904, &previous) == 4);
  assert(previous == 904);
  assert(diag_counter_delta(2, &previous) == 0);
  assert(previous == 2);

  uint64_t relative = 0;
  assert(diag_counter_relative(120, 100, &relative) && relative == 20);
  assert(!diag_counter_relative(99, 100, &relative));
  assert(diag_window_counts_match(20, 19, 1));
  assert(!diag_window_counts_match(20, 18, 1));

  assert(diag_gpio_policy_error(27, false) == NULL);
  assert(diag_gpio_policy_error(34, false) == NULL);
  assert(diag_gpio_policy_error(34, true) != NULL);
  assert(diag_gpio_policy_error(37, false) != NULL);
  assert(diag_gpio_policy_error(38, false) != NULL);
  assert(diag_gpio_policy_error(6, false) != NULL);
  assert(diag_gpio_policy_error(12, false) != NULL);
  assert(diag_gpio_policy_error(3, false) != NULL);
  assert(diag_gpio_policy_error(40, false) != NULL);
}

typedef struct {
  diag_measurement_t measurement;
  int64_t now_us;
  bool pulse_before_copy;
  int64_t pulse_before_copy_us;
  bool pulse_before_clock;
  int64_t pulse_before_clock_us;
} read_race_context_t;

static bool race_copy_measurement(void *opaque, diag_measurement_t *out) {
  read_race_context_t *context = opaque;
  if (!context || !out) return false;
  if (context->pulse_before_copy) {
    context->pulse_before_copy = false;
    assert(diag_measurement_record(&context->measurement,
                                   context->pulse_before_copy_us));
  }
  *out = context->measurement;
  return true;
}

static int64_t race_read_clock(void *opaque) {
  read_race_context_t *context = opaque;
  assert(context);
  if (context->pulse_before_clock) {
    context->pulse_before_clock = false;
    assert(diag_measurement_record(&context->measurement,
                                   context->pulse_before_clock_us));
  }
  return context->now_us;
}

static bool read_consistent(read_race_context_t *context,
                            diag_measurement_snapshot_t *snapshot) {
  return diag_measurement_read_consistent(race_copy_measurement,
                                          race_read_clock, context, snapshot);
}

static void test_timestamp_snapshot_race_and_resume(void) {
  const int64_t timeout_us = 500;
  diag_measurement_snapshot_t snapshot;

  /* Reproduce the former order: the caller samples time, then a pulse arrives
   * before it copies the ISR-updated state. The explicit old timestamp is
   * rejected by the underlying reader. */
  read_race_context_t old_order = {0};
  diag_measurement_reset(&old_order.measurement, 0, timeout_us, 0);
  old_order.pulse_before_copy = true;
  old_order.pulse_before_copy_us = 101;
  diag_measurement_t copied;
  assert(race_copy_measurement(&old_order, &copied));
  assert(!diag_measurement_read(&copied, 100, &snapshot));

  /* The same interleaving through the firmware's shared read helper takes the
   * reference after copying, so the first-pulse state remains valid. */
  read_race_context_t context = {0};
  diag_measurement_reset(&context.measurement, 0, timeout_us, 0);
  context.now_us = 102;
  context.pulse_before_copy = true;
  context.pulse_before_copy_us = 101;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.read_time_us == 102);
  assert(snapshot.raw_count == 1 && snapshot.accepted_count == 1);
  assert(snapshot.raw_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.since_last_event_us == 1);
  assert(snapshot.since_last_filtered_event_us == 1);
  assert(snapshot.since_last_event_us >= 0);

  /* A second event produces a valid period and frequency in the same helper. */
  context.now_us = 202;
  context.pulse_before_copy = true;
  context.pulse_before_copy_us = 201;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_count == 2 && snapshot.accepted_count == 2);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 100);
  assert(snapshot.filtered_period_valid && snapshot.filtered_period_us == 100);
  assert(fabs(snapshot.raw_frequency_hz - 10000.0) < 1e-12);
  assert(fabs(snapshot.filtered_frequency_hz - 10000.0) < 1e-12);

  /* An interrupt can also occur after the copied snapshot and before the
   * clock read. That snapshot remains internally valid; the next read sees
   * the new event. */
  context.now_us = 302;
  context.pulse_before_clock = true;
  context.pulse_before_clock_us = 302;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_count == 2 && snapshot.last_event_us == 201);
  assert(snapshot.read_time_us == 302 && snapshot.since_last_event_us == 101);
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 100);
  context.now_us = 303;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_count == 3 && snapshot.last_event_us == 302);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 101);
  assert(snapshot.since_last_event_us == 1);

  /* Repeated event/copy/clock interleavings stay monotonic and never produce
   * a negative age or a transient invalid-state result. */
  uint64_t previous_count = snapshot.raw_count;
  for (int64_t i = 1; i <= 1000; ++i) {
    const int64_t event_us = 302 + i * 100;
    context.now_us = event_us + 1;
    context.pulse_before_copy = true;
    context.pulse_before_copy_us = event_us;
    assert(read_consistent(&context, &snapshot));
    assert(snapshot.raw_count == previous_count + 1);
    assert(snapshot.accepted_count == snapshot.raw_count);
    assert(snapshot.raw_period_valid && snapshot.raw_period_us == 100);
    assert(fabs(snapshot.raw_frequency_hz - 10000.0) < 1e-12);
    assert(snapshot.since_last_event_us == 1);
    assert(snapshot.since_last_event_us >= 0);
    previous_count = snapshot.raw_count;
  }

  /* Timeout invalidates the historical period/frequency. The first resumed
   * event only reanchors time; the second provides a fresh period while the
   * cumulative pulse count remains monotonic. */
  const int64_t last_event_us = snapshot.last_event_us;
  context.now_us = last_event_us + timeout_us;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_state == DIAG_STATE_TIMEOUT);
  assert(snapshot.filtered_state == DIAG_STATE_TIMEOUT);
  assert(!snapshot.raw_period_valid && !snapshot.filtered_period_valid);
  assert(snapshot.raw_period_us == 0 && snapshot.filtered_period_us == 0);
  assert(snapshot.raw_frequency_hz == 0.0 &&
         snapshot.filtered_frequency_hz == 0.0);

  const uint64_t count_before_resume = snapshot.raw_count;
  context.pulse_before_copy = true;
  context.pulse_before_copy_us = last_event_us + timeout_us + 1;
  context.now_us = context.pulse_before_copy_us + 1;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_count == count_before_resume + 1);
  assert(snapshot.raw_state == DIAG_STATE_FIRST_EVENT);
  assert(snapshot.filtered_state == DIAG_STATE_FIRST_EVENT);
  assert(!snapshot.raw_period_valid && !snapshot.filtered_period_valid);

  context.pulse_before_copy = true;
  context.pulse_before_copy_us += 100;
  context.now_us = context.pulse_before_copy_us + 1;
  assert(read_consistent(&context, &snapshot));
  assert(snapshot.raw_count == count_before_resume + 2);
  assert(snapshot.raw_state == DIAG_STATE_MEASURING);
  assert(snapshot.filtered_state == DIAG_STATE_MEASURING);
  assert(snapshot.raw_period_valid && snapshot.raw_period_us == 100);
  assert(snapshot.filtered_period_valid && snapshot.filtered_period_us == 100);
  assert(snapshot.raw_frequency_hz == 10000.0);
  assert(snapshot.read_time_us >= snapshot.last_event_us);
  assert(snapshot.since_last_event_us >= 0);
}

int main(void) {
  test_edge_selection();
  test_period_frequency_rpm();
  test_first_timeout_resume_and_no_history();
  test_filter_and_raw_count();
  test_synthetic_rates_and_window_edges();
  test_restart_resets_state();
  test_filter_change_preserves_filtered_total();
  test_independent_raw_and_filtered_states();
  test_counter_baselines_and_gpio_policy();
  test_timestamp_snapshot_race_and_resume();
  puts("PASS: 10 deterministic diagnostic logic test groups");
  return 0;
}
