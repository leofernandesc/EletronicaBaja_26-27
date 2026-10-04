#include "speed_measurement.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static const int64_t timeout_us = 1000000;

static bool read_speed(const speed_measurement_t *state, int64_t now_us,
                       float calibration, float *speed) {
  return speed_measurement_read(state, now_us, timeout_us, calibration, speed);
}

static void test_startup_and_stop(void) {
  speed_measurement_t state = {.started_us = 100};
  float speed = -1;
  assert(!read_speed(&state, 100, 0.5f, &speed));
  assert(!read_speed(&state, 1000099, 0.5f, &speed));
  assert(read_speed(&state, 1000100, 0.5f, &speed));
  assert(speed == 0.0f);

  assert(speed_measurement_pulse(&state, 2000000, timeout_us, 0));
  assert(!read_speed(&state, 2000000, 0.5f, &speed));
  assert(speed_measurement_pulse(&state, 2100000, timeout_us, 0));
  assert(read_speed(&state, 2100000, 0.5f, &speed));
  assert(fabsf(speed - 5.0f) < 0.0001f);
  assert(read_speed(&state, 3099999, 0.5f, &speed));
  assert(speed == 5.0f);
  assert(read_speed(&state, 3100000, 0.5f, &speed));
  assert(speed == 0.0f);

  // Reads do not reset state; a stopped sensor restarts correctly even when
  // the logger did not request a reading during the gap.
  assert(speed_measurement_pulse(&state, 4000000, timeout_us, 0));
  assert(!read_speed(&state, 4000000, 0.5f, &speed));
  assert(speed_measurement_pulse(&state, 4200000, timeout_us, 0));
  assert(read_speed(&state, 4200000, 0.5f, &speed));
  assert(speed == 2.5f);
}

static void test_uncalibrated_and_independent_instances(void) {
  speed_measurement_t a = {0}, b = {0};
  float speed;
  assert(speed_measurement_pulse(&a, 0, timeout_us, 0));
  assert(speed_measurement_pulse(&a, 200000, timeout_us, 0));
  assert(speed_measurement_pulse(&b, 50000, timeout_us, 0));
  assert(speed_measurement_pulse(&b, 450000, timeout_us, 0));
  assert(read_speed(&a, 450000, 0.25f, &speed) && speed == 1.25f);
  assert(read_speed(&b, 450000, 0.25f, &speed) && speed == 0.625f);
  assert(!read_speed(&a, 450000, 0.0f, &speed));
  assert(!read_speed(&a, 2000000, 0.0f, &speed));
  assert(!read_speed(&a, 450000, NAN, &speed));
  assert(!read_speed(&a, 450000, FLT_MAX, &speed));
  assert(a.pulse_count == 2 && b.pulse_count == 2);
}

static void test_pulse_rejection(void) {
  speed_measurement_t state = {0};
  assert(speed_measurement_pulse(&state, 1000, timeout_us, 100));
  assert(!speed_measurement_pulse(&state, 1099, timeout_us, 100));
  assert(!speed_measurement_pulse(&state, 1000, timeout_us, 100));
  assert(!speed_measurement_pulse(&state, 999, timeout_us, 100));
  assert(state.last_pulse_us == 1000 && state.pulse_count == 1);
  assert(speed_measurement_pulse(&state, 1100, timeout_us, 100));
  assert(state.period_us == 100 && state.pulse_count == 2);
  assert(!speed_measurement_pulse(&state, 1199, timeout_us, 100));
  float speed;
  assert(read_speed(&state, 1001099, 0.1f, &speed) && speed > 0);
  assert(read_speed(&state, 1001100, 0.1f, &speed) && speed == 0);
}

static void test_slow_pulses_and_long_uptime(void) {
  // More than a 200 ms logger interval between edges must still measure speed.
  speed_measurement_t state = {.started_us = INT64_C(5000000000)};
  int64_t first = state.started_us + 100;
  assert(speed_measurement_pulse(&state, first, timeout_us, 0));
  assert(speed_measurement_pulse(&state, first + 800000, timeout_us, 0));
  float speed;
  assert(read_speed(&state, first + 900000, 0.8f, &speed));
  assert(fabsf(speed - 1.0f) < 0.0001f);
  // The exact timeout boundary restarts the period measurement.
  assert(speed_measurement_pulse(&state, first + 1800000, timeout_us, 0));
  assert(!read_speed(&state, first + 1800000, 0.8f, &speed));
  state.pulse_count = UINT32_MAX;
  assert(speed_measurement_pulse(&state, first + 1900000, timeout_us, 0));
  assert(state.pulse_count == UINT64_C(4294967296));
}

int main(void) {
  test_startup_and_stop();
  test_uncalibrated_and_independent_instances();
  test_pulse_rejection();
  test_slow_pulses_and_long_uptime();
  puts("speed measurement tests passed");
  return 0;
}
