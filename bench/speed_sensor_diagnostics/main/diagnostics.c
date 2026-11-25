#include "diagnostics.h"

#include "speed_sensor.h"

#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static diag_measurement_t s_measurement;
static portMUX_TYPE s_measurement_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_capture_active;
static gpio_num_t s_capture_gpio = GPIO_NUM_MAX;
static uint32_t s_capture_timeout_ms;

static const char *reset_reason_name(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER";
  }
}

static esp_err_t diag_prepare_gpio_isr_service(void) {
  /*
   * Reuse the extracted component to install the shared ISR service and keep
   * its lifecycle bookkeeping valid. The temporary GPIO25 handler is removed
   * immediately; no sensor is connected to this diagnostic bootstrap pin.
   */
  speed_sensor_config_t config = SPEED_SENSOR_CONFIG_DEFAULT();
  config.gpio = GPIO_NUM_25;
  config.stop_timeout_ms = 5000;
  config.distance_per_pulse_m = 0.0f;
  config.min_pulse_interval_us = 0;
  config.pulse_edge = GPIO_INTR_POSEDGE;
  speed_sensor_handle_t handle = NULL;
  esp_err_t err = speed_sensor_init(&config, &handle);
  if (err != ESP_OK) return err;
  err = speed_sensor_deinit(handle);
  return err;
}

static void IRAM_ATTR diagnostic_gpio_isr(void *arg) {
  (void)arg;
  const int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&s_measurement_lock);
  diag_measurement_record(&s_measurement, now_us);
  portEXIT_CRITICAL_ISR(&s_measurement_lock);
}

diag_config_t diag_config_from_menuconfig(void) {
  diag_edge_mode_t edge = DIAG_EDGE_RISING;
#if CONFIG_DIAG_DEFAULT_EDGE_FALLING
  edge = DIAG_EDGE_FALLING;
#endif
  diag_pull_mode_t pull = DIAG_PULL_DISABLED;
#if CONFIG_DIAG_PULL_UP
  pull = DIAG_PULL_UP;
#elif CONFIG_DIAG_PULL_DOWN
  pull = DIAG_PULL_DOWN;
#endif
  uint32_t generator_frequency = 10;
#if CONFIG_DIAG_GENERATOR_1HZ
  generator_frequency = 1;
#elif CONFIG_DIAG_GENERATOR_5HZ
  generator_frequency = 5;
#elif CONFIG_DIAG_GENERATOR_100HZ
  generator_frequency = 100;
#elif CONFIG_DIAG_GENERATOR_500HZ
  generator_frequency = 500;
#elif CONFIG_DIAG_GENERATOR_1000HZ
  generator_frequency = 1000;
#elif CONFIG_DIAG_GENERATOR_20HZ
  generator_frequency = 20;
#endif
  uint32_t pcnt_filter_ns = 0;
#ifdef CONFIG_DIAG_T7_PCNT_GLITCH_FILTER_NS
  pcnt_filter_ns = CONFIG_DIAG_T7_PCNT_GLITCH_FILTER_NS;
#endif
  bool pcnt_enabled = false;
#if CONFIG_DIAG_T7_ENABLE_PCNT
  pcnt_enabled = true;
#endif
  bool rpm_confirmed = false;
#if CONFIG_DIAG_RPM_RELATIONSHIP_CONFIRMED
  rpm_confirmed = true;
#endif
  bool t3_use_driver = false;
#if CONFIG_DIAG_T3_USE_SPEED_SENSOR_DRIVER
  t3_use_driver = true;
#endif
  return (diag_config_t){
      .input_gpio = (gpio_num_t)CONFIG_DIAG_INPUT_GPIO,
      .generator_gpio = (gpio_num_t)CONFIG_DIAG_GENERATOR_GPIO,
      .sample_interval_ms = CONFIG_DIAG_SAMPLE_INTERVAL_MS,
      .timeout_ms = CONFIG_DIAG_STOP_TIMEOUT_MS,
      .min_pulse_interval_us = CONFIG_DIAG_FILTER_MIN_INTERVAL_US,
      .pulses_per_revolution = CONFIG_DIAG_PULSES_PER_REVOLUTION,
      .generator_frequency_hz = generator_frequency,
      .t4_rapid_edge_warning_us = CONFIG_DIAG_T4_RAPID_EDGE_WARNING_US,
      .pcnt_glitch_filter_ns = pcnt_filter_ns,
      .edge_mode = edge,
      .pull_mode = pull,
      .rpm_relationship_confirmed = rpm_confirmed,
      .pcnt_enabled = pcnt_enabled,
      .t3_use_speed_sensor_driver = t3_use_driver,
  };
}

esp_err_t diag_validate_input_gpio(gpio_num_t gpio) {
  return diag_validate_input_gpio_config(gpio, DIAG_PULL_DISABLED);
}

const char *diag_gpio_input_error(gpio_num_t gpio, diag_pull_mode_t pull) {
  if (!GPIO_IS_VALID_GPIO(gpio)) return "GPIO nao existe ou nao esta disponivel neste ESP32";
  const char *policy_error = diag_gpio_policy_error((int)gpio, false);
  if (policy_error) return policy_error;
  if (gpio >= GPIO_NUM_34 && gpio <= GPIO_NUM_39 &&
      pull != DIAG_PULL_DISABLED)
    return "GPIO 34-39 nao possuem pull-up/pull-down interno";
  return NULL;
}

const char *diag_gpio_output_error(gpio_num_t gpio) {
  if (!GPIO_IS_VALID_GPIO(gpio)) return "GPIO nao existe ou nao esta disponivel neste ESP32";
  if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) return "GPIO nao suporta saida digital";
  return diag_gpio_policy_error((int)gpio, true);
}

esp_err_t diag_validate_input_gpio_config(gpio_num_t gpio,
                                          diag_pull_mode_t pull) {
  return diag_gpio_input_error(gpio, pull) ? ESP_ERR_INVALID_ARG : ESP_OK;
}

esp_err_t diag_validate_output_gpio(gpio_num_t gpio) {
  return diag_gpio_output_error(gpio) ? ESP_ERR_INVALID_ARG : ESP_OK;
}

gpio_int_type_t diag_gpio_intr_type(diag_edge_mode_t edge) {
  switch (edge) {
    case DIAG_EDGE_RISING: return GPIO_INTR_POSEDGE;
    case DIAG_EDGE_FALLING: return GPIO_INTR_NEGEDGE;
    case DIAG_EDGE_BOTH: return GPIO_INTR_ANYEDGE;
    default: return GPIO_INTR_DISABLE;
  }
}

const char *diag_edge_name(diag_edge_mode_t edge) {
  switch (edge) {
    case DIAG_EDGE_RISING: return "RISING";
    case DIAG_EDGE_FALLING: return "FALLING";
    case DIAG_EDGE_BOTH: return "BOTH";
    default: return "INVALID";
  }
}

const char *diag_pull_name(diag_pull_mode_t pull) {
  switch (pull) {
    case DIAG_PULL_DISABLED: return "DISABLED";
    case DIAG_PULL_DOWN: return "DOWN";
    case DIAG_PULL_UP: return "UP";
    default: return "INVALID";
  }
}

const char *diag_state_name(diag_measurement_state_t state) {
  switch (state) {
    case DIAG_STATE_UNAVAILABLE: return "INDISPONIVEL";
    case DIAG_STATE_NO_EVENTS: return "SEM_PULSOS";
    case DIAG_STATE_FIRST_EVENT: return "PRIMEIRO_EVENTO";
    case DIAG_STATE_WAITING_PERIOD: return "AGUARDANDO_PERIODO";
    case DIAG_STATE_MEASURING: return "MEDINDO";
    case DIAG_STATE_TIMEOUT: return "TIMEOUT";
    case DIAG_STATE_ERROR: return "ERRO";
    default: return "ERRO";
  }
}

esp_err_t diag_capture_start(const diag_config_t *config,
                             diag_edge_mode_t edge,
                             diag_pull_mode_t pull,
                             uint32_t min_interval_us) {
  if (!config || s_capture_active ||
      diag_validate_input_gpio_config(config->input_gpio, pull) != ESP_OK ||
      (edge != DIAG_EDGE_RISING && edge != DIAG_EDGE_FALLING &&
       edge != DIAG_EDGE_BOTH) || config->timeout_ms == 0 ||
      min_interval_us >= (uint64_t)config->timeout_ms * 1000U)
    return ESP_ERR_INVALID_ARG;

  esp_err_t err = diag_prepare_gpio_isr_service();
  if (err != ESP_OK) return err;

  gpio_config_t pin_config = {
      .pin_bit_mask = 1ULL << config->input_gpio,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = pull == DIAG_PULL_UP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
      .pull_down_en = pull == DIAG_PULL_DOWN ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  err = gpio_config(&pin_config);
  if (err != ESP_OK) return err;

  const int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_measurement_lock);
  diag_measurement_reset(&s_measurement, now_us,
                         (int64_t)config->timeout_ms * 1000,
                         min_interval_us);
  portEXIT_CRITICAL(&s_measurement_lock);

  err = gpio_isr_handler_add(config->input_gpio, diagnostic_gpio_isr, NULL);
  if (err != ESP_OK) {
    gpio_reset_pin(config->input_gpio);
    return err;
  }
  err = gpio_set_intr_type(config->input_gpio, diag_gpio_intr_type(edge));
  if (err != ESP_OK) {
    gpio_isr_handler_remove(config->input_gpio);
    gpio_reset_pin(config->input_gpio);
    return err;
  }
  s_capture_gpio = config->input_gpio;
  s_capture_timeout_ms = config->timeout_ms;
  s_capture_active = true;
  return ESP_OK;
}

esp_err_t diag_capture_stop(void) {
  if (!s_capture_active) return ESP_OK;
  esp_err_t err = gpio_intr_disable(s_capture_gpio);
  if (err != ESP_OK) return err;
  err = gpio_isr_handler_remove(s_capture_gpio);
  if (err != ESP_OK) return err;
  err = gpio_reset_pin(s_capture_gpio);
  if (err != ESP_OK) return err;
  s_capture_active = false;
  s_capture_gpio = GPIO_NUM_MAX;
  return ESP_OK;
}

esp_err_t diag_capture_clear(void) {
  if (!s_capture_active) return ESP_ERR_INVALID_STATE;
  const int64_t now_us = esp_timer_get_time();
  portENTER_CRITICAL(&s_measurement_lock);
  diag_measurement_reset(&s_measurement, now_us,
                         (int64_t)s_capture_timeout_ms * 1000,
                         s_measurement.min_interval_us);
  portEXIT_CRITICAL(&s_measurement_lock);
  return ESP_OK;
}

esp_err_t diag_capture_set_filter(uint32_t min_interval_us) {
  if (!s_capture_active || min_interval_us >= (uint64_t)s_capture_timeout_ms * 1000U)
    return ESP_ERR_INVALID_STATE;
  portENTER_CRITICAL(&s_measurement_lock);
  bool ok = diag_measurement_set_filter(&s_measurement, esp_timer_get_time(),
                                       min_interval_us);
  portEXIT_CRITICAL(&s_measurement_lock);
  return ok ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static bool diag_copy_capture_measurement(void *context,
                                          diag_measurement_t *measurement) {
  (void)context;
  if (!s_capture_active || !measurement) return false;
  portENTER_CRITICAL(&s_measurement_lock);
  *measurement = s_measurement;
  portEXIT_CRITICAL(&s_measurement_lock);
  return true;
}

static int64_t diag_capture_clock(void *context) {
  (void)context;
  return esp_timer_get_time();
}

esp_err_t diag_capture_read(diag_capture_snapshot_t *snapshot) {
  if (!s_capture_active || !snapshot) return ESP_ERR_INVALID_STATE;
  diag_measurement_snapshot_t measured;
  if (!diag_measurement_read_consistent(diag_copy_capture_measurement,
                                        diag_capture_clock, NULL, &measured))
    return ESP_ERR_INVALID_STATE;
  *snapshot = (diag_capture_snapshot_t){
      .read_time_us = measured.read_time_us,
      .raw_count = measured.raw_count,
      .accepted_count = measured.accepted_count,
      .rejected_count = measured.rejected_count,
      .last_event_us = measured.last_event_us,
      .since_last_event_us = measured.since_last_event_us,
      .filtered_last_event_us = measured.filtered_last_event_us,
      .since_last_filtered_event_us = measured.since_last_filtered_event_us,
      .raw_period_us = measured.raw_period_us,
      .filtered_period_us = measured.filtered_period_us,
      .raw_frequency_hz = measured.raw_frequency_hz,
      .filtered_frequency_hz = measured.filtered_frequency_hz,
      .raw_state = measured.raw_state,
      .filtered_state = measured.filtered_state,
      .raw_period_valid = measured.raw_period_valid,
      .filtered_period_valid = measured.filtered_period_valid,
      .counter_overflow = measured.counter_overflow,
  };
  return ESP_OK;
}

esp_err_t diag_capture_read_level(int *level) {
  if (!s_capture_active || !level) return ESP_ERR_INVALID_STATE;
  int value = gpio_get_level(s_capture_gpio);
  if (value < 0) return ESP_FAIL;
  *level = value;
  return ESP_OK;
}

bool diag_capture_is_active(void) { return s_capture_active; }

int64_t diag_uptime_us(const diag_runtime_t *runtime, int64_t now_us) {
  if (!runtime || now_us < runtime->boot_started_us) return 0;
  return now_us - runtime->boot_started_us;
}

void diag_print_boot_report(uint32_t boot_id) {
  const esp_app_desc_t *app = esp_app_get_description();
  esp_chip_info_t chip = {0};
  esp_chip_info(&chip);
  printf("\n[BOOT] firmware=%s boot=%08" PRIx32 " version=%s build=%s %s\r\n",
         CONFIG_DIAG_FIRMWARE_ID, boot_id, app->version, app->date, app->time);
  printf("[BOOT] idf=%s chip=ESP32 revision=%u cores=%u features=0x%" PRIx32 " reset=%s(%d)\r\n",
         esp_get_idf_version(), chip.revision, chip.cores, chip.features,
         reset_reason_name(esp_reset_reason()), esp_reset_reason());
  printf("[BOOT] app=%s elf_sha256=%.16s gpio_in=%d gpio_gen=%d baud=115200\r\n",
         app->project_name, esp_app_get_elf_sha256_str(),
         CONFIG_DIAG_INPUT_GPIO, CONFIG_DIAG_GENERATOR_GPIO);
  printf("[BOOT] compiled=%s %s boot_counter=PERSISTENCE_NOT_CONFIGURED\r\n",
         __DATE__, __TIME__);
}

void diag_print_test_header(const diag_runtime_t *runtime,
                            const diag_test_t *test,
                            const char *init_state,
                            const char *mode_config) {
  const esp_app_desc_t *app = esp_app_get_description();
  const char *pull = diag_pull_name(runtime->config.pull_mode);
  if (!strcmp(test->id, "T0") || !strcmp(test->id, "T5")) pull = "N/A";
  if (!strcmp(test->id, "T1") || !strcmp(test->id, "T3") ||
      !strcmp(test->id, "T6")) pull = "DISABLED";
  printf("\n[%s] START firmware=%s app=%s version=%s idf=%s build=%s_%s sha256=%.16s boot=%08" PRIx32 " run=%" PRIu32 "\r\n",
         test->id, CONFIG_DIAG_FIRMWARE_ID, app->project_name, app->version,
         esp_get_idf_version(), app->date, app->time,
         esp_app_get_elf_sha256_str(),
         runtime->boot_id, runtime->run_id);
  printf("[%s] config gpio_in=%d gpio_gen=%d edge=%s pull=%s interval_ms=%" PRIu32 " timeout_ms=%" PRIu32 " ppr=%" PRIu32 " filter_us=%" PRIu32 " pcnt=%d mode=%s init=%s\r\n",
         test->id, runtime->config.input_gpio, runtime->config.generator_gpio,
         diag_edge_name(runtime->config.edge_mode),
         pull,
         runtime->config.sample_interval_ms, runtime->config.timeout_ms,
         runtime->config.pulses_per_revolution,
         runtime->config.min_pulse_interval_us, runtime->config.pcnt_enabled,
         mode_config ? mode_config : "-", init_state ? init_state : "ERRO");
}

void diag_print_prefix(const diag_runtime_t *runtime, const char *test_id,
                       const char *mode_config, int64_t now_us) {
  printf("[%s] boot=%08" PRIx32 " run=%" PRIu32 " up_ms=%" PRId64 " cfg=%s ",
         test_id, runtime->boot_id, runtime->run_id,
         diag_uptime_us(runtime, now_us) / 1000,
         mode_config ? mode_config : "-");
}
