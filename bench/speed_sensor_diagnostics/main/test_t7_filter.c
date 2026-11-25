#include "diagnostics.h"

#include "driver/pulse_cnt.h"
#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>

static pcnt_unit_handle_t s_pcnt_unit;
static pcnt_channel_handle_t s_pcnt_channel;
static uint64_t s_previous_raw;
static uint64_t s_previous_filtered;
static uint64_t s_raw_baseline;
static uint64_t s_filtered_baseline;
static uint64_t s_previous_pcnt_relative;
static int s_pcnt_baseline;
static int s_pcnt_previous_value;
static bool s_pcnt_overflow;
static bool s_isr_overflow;
static esp_err_t s_pcnt_init_error;
static esp_err_t s_pcnt_cleanup_error;

static esp_err_t pcnt_cleanup(void) {
  esp_err_t result = ESP_OK;
  if (s_pcnt_unit) {
    esp_err_t err = pcnt_unit_stop(s_pcnt_unit);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) result = err;
    err = pcnt_unit_disable(s_pcnt_unit);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE && result == ESP_OK)
      result = err;
  }
  if (s_pcnt_channel) {
    esp_err_t err = pcnt_del_channel(s_pcnt_channel);
    if (err == ESP_OK) s_pcnt_channel = NULL;
    else if (result == ESP_OK) result = err;
  }
  if (s_pcnt_unit && !s_pcnt_channel) {
    esp_err_t err = pcnt_del_unit(s_pcnt_unit);
    if (err == ESP_OK) s_pcnt_unit = NULL;
    else if (result == ESP_OK) result = err;
  }
  return result;
}

static esp_err_t pcnt_start(const diag_config_t *config) {
  s_pcnt_unit = NULL;
  s_pcnt_channel = NULL;
  pcnt_unit_config_t unit_config = {
      .low_limit = -32768,
      .high_limit = 32767,
      .intr_priority = 0,
      .flags.accum_count = 1,
  };
  esp_err_t err = pcnt_new_unit(&unit_config, &s_pcnt_unit);
  if (err != ESP_OK) return err;
  err = pcnt_unit_add_watch_point(s_pcnt_unit, 32767);
  if (err != ESP_OK) goto fail;
  err = pcnt_unit_add_watch_point(s_pcnt_unit, -32768);
  if (err != ESP_OK) goto fail;
  if (config->pcnt_glitch_filter_ns) {
    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = config->pcnt_glitch_filter_ns,
    };
    err = pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter);
    if (err != ESP_OK) goto fail;
  }
  pcnt_chan_config_t channel_config = {
      .edge_gpio_num = config->input_gpio,
      .level_gpio_num = -1,
  };
  err = pcnt_new_channel(s_pcnt_unit, &channel_config, &s_pcnt_channel);
  if (err != ESP_OK) goto fail;
  pcnt_channel_edge_action_t positive = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  pcnt_channel_edge_action_t negative = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  if (config->edge_mode == DIAG_EDGE_RISING || config->edge_mode == DIAG_EDGE_BOTH)
    positive = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
  if (config->edge_mode == DIAG_EDGE_FALLING || config->edge_mode == DIAG_EDGE_BOTH)
    negative = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
  err = pcnt_channel_set_edge_action(s_pcnt_channel, positive, negative);
  if (err != ESP_OK) goto fail;
  err = pcnt_unit_enable(s_pcnt_unit);
  if (err != ESP_OK) goto fail;
  err = pcnt_unit_clear_count(s_pcnt_unit);
  if (err != ESP_OK) goto fail;
  err = pcnt_unit_start(s_pcnt_unit);
  if (err != ESP_OK) goto fail;
  return ESP_OK;
fail:
  s_pcnt_cleanup_error = pcnt_cleanup();
  return err;
}

esp_err_t test_t7_start(diag_runtime_t *runtime) {
  if (runtime->config.edge_mode == DIAG_EDGE_BOTH)
    return ESP_ERR_INVALID_ARG;
  s_previous_raw = 0;
  s_previous_filtered = 0;
  s_raw_baseline = 0;
  s_filtered_baseline = 0;
  s_previous_pcnt_relative = 0;
  s_pcnt_baseline = 0;
  s_pcnt_previous_value = 0;
  s_pcnt_overflow = false;
  s_isr_overflow = false;
  s_pcnt_init_error = ESP_OK;
  s_pcnt_cleanup_error = ESP_OK;
  esp_err_t err = diag_capture_start(&runtime->config,
                                    runtime->config.edge_mode,
                                    runtime->config.pull_mode,
                                    runtime->config.min_pulse_interval_us);
  if (err != ESP_OK) return err;
  if (runtime->config.pcnt_enabled) {
    err = pcnt_start(&runtime->config);
    if (err != ESP_OK) {
      s_pcnt_init_error = err;
      runtime->init_status = "PARTIAL_PCNT_UNAVAILABLE";
      runtime->init_error = err;
      if (s_pcnt_cleanup_error != ESP_OK) {
        diag_print_prefix(runtime, "T7", "STARTUP_CLEANUP", esp_timer_get_time());
        printf("PCNT_CLEANUP_ERROR=%s code=%d\r\n",
               esp_err_to_name(s_pcnt_cleanup_error),
               (int)s_pcnt_cleanup_error);
        esp_err_t capture_err = diag_capture_stop();
        if (capture_err != ESP_OK) return capture_err;
        return s_pcnt_cleanup_error;
      }
    }
  }

  diag_capture_snapshot_t baseline;
  err = diag_capture_read(&baseline);
  if (err != ESP_OK) {
    esp_err_t cleanup_err = pcnt_cleanup();
    esp_err_t capture_err = diag_capture_stop();
    if (cleanup_err != ESP_OK) return cleanup_err;
    if (capture_err != ESP_OK) return capture_err;
    return err;
  }
  s_raw_baseline = baseline.raw_count;
  s_filtered_baseline = baseline.accepted_count;
  s_previous_raw = baseline.raw_count;
  s_previous_filtered = baseline.accepted_count;
  if (s_pcnt_unit && s_pcnt_init_error == ESP_OK) {
    int pcnt_value = 0;
    err = pcnt_unit_get_count(s_pcnt_unit, &pcnt_value);
    if (err == ESP_OK) {
      s_pcnt_baseline = pcnt_value;
      s_pcnt_previous_value = pcnt_value;
      if (pcnt_value < 0) s_pcnt_overflow = true;
    } else {
      s_pcnt_init_error = err;
      runtime->init_status = "PARTIAL_PCNT_BASELINE_UNAVAILABLE";
      runtime->init_error = err;
    }
  }
  return ESP_OK;
}

esp_err_t test_t7_set_filter(diag_runtime_t *runtime, uint32_t min_interval_us) {
  esp_err_t err = diag_capture_set_filter(min_interval_us);
  if (err == ESP_OK) {
    runtime->config.min_pulse_interval_us = min_interval_us;
    diag_capture_snapshot_t snapshot;
    err = diag_capture_read(&snapshot);
    s_previous_filtered = err == ESP_OK ? snapshot.accepted_count : UINT64_MAX;
  }
  return err;
}

esp_err_t test_t7_update(diag_runtime_t *runtime, int64_t now_us) {
  (void)now_us;
  diag_capture_snapshot_t snapshot;
  esp_err_t err = diag_capture_read(&snapshot);
  if (err != ESP_OK) return err;
  now_us = snapshot.read_time_us;
  uint64_t raw_relative = 0;
  uint64_t filtered_relative = 0;
  if (!diag_counter_relative(snapshot.raw_count, s_raw_baseline,
                             &raw_relative))
    s_isr_overflow = true;
  if (!diag_counter_relative(snapshot.accepted_count, s_filtered_baseline,
                             &filtered_relative))
    s_isr_overflow = true;
  uint64_t raw_delta = diag_counter_delta(snapshot.raw_count, &s_previous_raw);
  uint64_t filtered_delta = diag_counter_delta(snapshot.accepted_count,
                                                &s_previous_filtered);
  uint64_t rejected = snapshot.rejected_count;
  double rejected_percent = snapshot.raw_count
      ? (double)rejected * 100.0 / (double)snapshot.raw_count : 0.0;
  int pcnt_value = 0;
  esp_err_t pcnt_read_error = ESP_ERR_INVALID_STATE;
  if (s_pcnt_init_error == ESP_OK && s_pcnt_unit)
    pcnt_read_error = pcnt_unit_get_count(s_pcnt_unit, &pcnt_value);
  bool pcnt_ok = pcnt_read_error == ESP_OK;
  uint64_t pcnt_relative = 0;
  uint64_t pcnt_delta = 0;
  if (pcnt_ok) {
    if (pcnt_value < 0 || pcnt_value < s_pcnt_previous_value ||
        pcnt_value < s_pcnt_baseline) {
      s_pcnt_overflow = true;
    } else {
      pcnt_relative = (uint64_t)(pcnt_value - s_pcnt_baseline);
      if (pcnt_relative < s_previous_pcnt_relative) {
        s_pcnt_overflow = true;
      } else {
        pcnt_delta = pcnt_relative - s_previous_pcnt_relative;
      }
    }
    s_pcnt_previous_value = pcnt_value;
    s_previous_pcnt_relative = pcnt_relative;
  }
  s_isr_overflow = s_isr_overflow || snapshot.counter_overflow;
  bool pcnt_compare_valid = pcnt_ok && !s_pcnt_overflow && !s_isr_overflow;
  bool pcnt_counts_match = pcnt_compare_valid &&
      diag_window_counts_match(pcnt_relative, raw_relative, 1);
  bool pcnt_delta_match = pcnt_compare_valid &&
      diag_window_counts_match(pcnt_delta, raw_delta, 1);
  bool rpm_valid = false;
  double frequency_hz = 0.0;
  double rpm = 0.0;
  if (snapshot.filtered_state == DIAG_STATE_MEASURING &&
      snapshot.filtered_period_valid)
    rpm_valid = diag_calculate_rpm(snapshot.filtered_period_us,
                                   runtime->config.pulses_per_revolution,
                                   runtime->config.rpm_relationship_confirmed,
                                   &frequency_hz, &rpm);
  diag_print_prefix(runtime, "T7", "RPM_REQUIRES_RPM_CONFIRM", now_us);
  printf("EDGE=%s RAW=%" PRIu64 " RAW_DELTA=%" PRIu64 " LAST_EVENT_US=",
         diag_edge_name(runtime->config.edge_mode), snapshot.raw_count, raw_delta);
  if (snapshot.raw_count) printf("%" PRId64, snapshot.last_event_us);
  else printf("--");
  printf(
         " RAW_BASELINE=%" PRIu64 " RAW_REL=%" PRIu64
         " FILTERED_BASELINE=%" PRIu64 " FILTERED=%" PRIu64
         " FILTERED_REL=%" PRIu64 " FILTERED_DELTA=%" PRIu64
         " REJECTED=%" PRIu64 " REJECTED_PERCENT=%.3f FILTER_MIN_US=%" PRIu32,
         s_raw_baseline, raw_relative, s_filtered_baseline,
         snapshot.accepted_count, filtered_relative,
         filtered_delta, rejected, rejected_percent,
         runtime->config.min_pulse_interval_us);
  if (!runtime->config.pcnt_enabled)
    printf(" PCNT=DISABLED");
  else if (s_pcnt_init_error != ESP_OK)
    printf(" PCNT=UNAVAILABLE PCNT_INIT=%s",
           esp_err_to_name(s_pcnt_init_error));
  else if (pcnt_ok) {
    printf(" PCNT_TOTAL=%d PCNT_BASELINE=%d PCNT_HW_LIMITS=-32768..32767"
           " PCNT_REL=",
           pcnt_value, s_pcnt_baseline);
    if (s_pcnt_overflow) printf("OVERFLOW");
    else printf("%" PRIu64, pcnt_relative);
    printf(" PCNT_DELTA=");
    if (s_pcnt_overflow) printf("OVERFLOW");
    else printf("%" PRIu64, pcnt_delta);
    printf(" PCNT_ISR_DELTA_DIFF=");
    if (pcnt_compare_valid) {
      if (pcnt_delta >= raw_delta)
        printf("+%" PRIu64, pcnt_delta - raw_delta);
      else
        printf("-%" PRIu64, raw_delta - pcnt_delta);
    } else {
      printf("--");
    }
    printf(" PCNT_ISR_DELTA_CHECK=%s",
           !pcnt_compare_valid ? "UNAVAILABLE_OVERFLOW_OR_READ_ERROR"
           : pcnt_delta_match ? "MATCH_WITH_1_EDGE_BOUNDARY_TOLERANCE"
                              : "DIVERGENCE_REVIEW_REQUIRED");
    printf(" PCNT_ISR_REL_DIFF=");
    if (pcnt_compare_valid) {
      if (pcnt_relative >= raw_relative)
        printf("+%" PRIu64, pcnt_relative - raw_relative);
      else
        printf("-%" PRIu64, raw_relative - pcnt_relative);
    } else {
      printf("--");
    }
    printf(" PCNT_ISR_COMPARE=%s",
           !pcnt_compare_valid ? "UNAVAILABLE_OVERFLOW_OR_READ_ERROR"
           : pcnt_counts_match ? "MATCH_WITH_1_EDGE_BOUNDARY_TOLERANCE"
                               : "DIVERGENCE_REVIEW_REQUIRED");
  }
  else
    printf(" PCNT=UNAVAILABLE PCNT_READ=%s",
           esp_err_to_name(pcnt_read_error));
  printf(" RAW_PERIOD_US=");
  if (snapshot.raw_period_valid) printf("%" PRId64, snapshot.raw_period_us);
  else printf("--");
  printf(" FILTERED_PERIOD_US=");
  if (snapshot.filtered_period_valid)
    printf("%" PRId64, snapshot.filtered_period_us);
  else
    printf("--");
  printf(" FILTERED_FREQ_HZ=");
  if (snapshot.filtered_period_valid)
    printf("%.6f", snapshot.filtered_frequency_hz);
  else
    printf("%s", snapshot.filtered_state == DIAG_STATE_TIMEOUT
                     ? "0.000000" : "--");
  printf(" RAW_FREQ_HZ=");
  if (snapshot.raw_period_valid)
    printf("%.6f", snapshot.raw_frequency_hz);
  else
    printf("%s", snapshot.raw_state == DIAG_STATE_TIMEOUT
                     ? "0.000000" : "--");
  printf(" PPR=%" PRIu32 " RPM=", runtime->config.pulses_per_revolution);
  if (rpm_valid)
    printf("%.3f", rpm);
  else if (runtime->config.rpm_relationship_confirmed &&
           snapshot.filtered_state == DIAG_STATE_TIMEOUT)
    printf("0.000");
  else
    printf("--");
  printf(" RAW_AGE_MS=%.3f FILTERED_AGE_MS=%.3f RAW_STATE=%s"
         " FILTERED_STATE=%s ISR_COUNTER_OVERFLOW=%d PCNT_OVERFLOW=%d"
         " RPM_RELATION=%s"
         " PCNT_GLITCH_FILTER_NS=%" PRIu32 "\r\n",
         (double)snapshot.since_last_event_us / 1000.0,
         (double)snapshot.since_last_filtered_event_us / 1000.0,
         diag_state_name(snapshot.raw_state),
         diag_state_name(snapshot.filtered_state), s_isr_overflow,
         s_pcnt_overflow,
         runtime->config.rpm_relationship_confirmed ? "CONFIRMED_BY_OPERATOR"
                                                    : "UNVERIFIED",
         runtime->config.pcnt_glitch_filter_ns);
  return ESP_OK;
}

esp_err_t test_t7_stop(diag_runtime_t *runtime) {
  (void)runtime;
  esp_err_t pcnt_err = pcnt_cleanup();
  esp_err_t capture_err = diag_capture_stop();
  return pcnt_err != ESP_OK ? pcnt_err : capture_err;
}
