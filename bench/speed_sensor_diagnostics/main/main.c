#include "diagnostics.h"

#include "esp_err.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define COMMAND_LENGTH 128

typedef struct {
  char text[COMMAND_LENGTH];
} command_t;

static QueueHandle_t s_command_queue;
static diag_runtime_t s_runtime;
static diag_test_t s_tests[] = {
    {"T0", "boot and MCU integrity", test_t0_start, test_t0_update, test_t0_stop},
    {"T1", "GPIO quiet input", test_t1_start, test_t1_update, test_t1_stop},
    {"T2", "digital edge validation", test_t2_start, test_t2_update, test_t2_stop},
    {"T3", "internal pulse generator", test_t3_start, test_t3_update, test_t3_stop},
    {"T4", "static sensor level", test_t4_start, test_t4_update, test_t4_stop},
    {"T5", "external scope observation", test_t5_start, test_t5_update, test_t5_stop},
    {"T6", "selected-edge acquisition", test_t6_start, test_t6_update, test_t6_stop},
    {"T7", "filter and optional RPM", test_t7_start, test_t7_update, test_t7_stop},
};
static unsigned s_test_index;
static bool s_test_active;
static bool s_cleanup_blocked;

static void serial_input_task(void *arg) {
  (void)arg;
  char line[COMMAND_LENGTH];
  for (;;) {
    if (!fgets(line, sizeof(line), stdin)) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    size_t length = strlen(line);
    while (length && (line[length - 1] == '\n' || line[length - 1] == '\r'))
      line[--length] = '\0';
    if (!length) continue;
    command_t command = {0};
    strncpy(command.text, line, sizeof(command.text) - 1);
    (void)xQueueSend(s_command_queue, &command, portMAX_DELAY);
  }
}

static void print_help(void) {
  printf("\n[CMD] T0..T7 | EDGE RISING|FALLING|BOTH | FREQ 1|5|10|20"
         "|100|500|1000 | CAPTURE DIAG|DRIVER | FILTER <us>"
         " | PPR <n> | RPM CONFIRM|OFF"
         " | MARK METAL|GAP | NOTE <text> | HELP\r\n");
}

static const char *test_gpio_preflight_error(unsigned index) {
  if (index == 0 || index == 5) return NULL;
  diag_pull_mode_t pull = s_runtime.config.pull_mode;
  if (index == 1 || index == 3 || index == 6) pull = DIAG_PULL_DISABLED;
  const char *error = diag_gpio_input_error(s_runtime.config.input_gpio, pull);
  if (error) return error;
  if (index == 3) {
    if (s_runtime.config.input_gpio == s_runtime.config.generator_gpio)
      return "GPIO de entrada e saida do gerador nao podem ser iguais";
    return diag_gpio_output_error(s_runtime.config.generator_gpio);
  }
  return NULL;
}

static esp_err_t stop_current_test(void) {
  if (!s_test_active) return ESP_OK;
  esp_err_t err = s_tests[s_test_index].stop(&s_runtime);
  s_test_active = false;
  if (err != ESP_OK) s_cleanup_blocked = true;
  return err;
}

static esp_err_t start_test(unsigned index) {
  if (index >= sizeof(s_tests) / sizeof(s_tests[0])) return ESP_ERR_INVALID_ARG;
  const char *gpio_error = test_gpio_preflight_error(index);
  if (gpio_error) {
    printf("[GPIO] boot=%08" PRIx32 " test=T%u INIT_BLOCKED=%s"
           " input_gpio=%d generator_gpio=%d; perifericos nao inicializados\r\n",
           s_runtime.boot_id, index, gpio_error, s_runtime.config.input_gpio,
           s_runtime.config.generator_gpio);
    return ESP_ERR_INVALID_ARG;
  }
  if (s_cleanup_blocked) {
    printf("[CTRL] boot=%08" PRIx32
           " mode_switch=BLOCKED resource_cleanup_failed; reboot required\r\n",
           s_runtime.boot_id);
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t stop_err = stop_current_test();
  if (stop_err != ESP_OK) {
    printf("[CTRL] boot=%08" PRIx32 " run=%" PRIu32
           " previous_test_stop=ERROR:%s\r\n",
           s_runtime.boot_id, s_runtime.run_id, esp_err_to_name(stop_err));
    return stop_err;
  }
  s_test_index = index;
  s_runtime.run_id++;
  s_runtime.init_status = NULL;
  s_runtime.init_error = ESP_OK;
  esp_err_t err = s_tests[index].start(&s_runtime);
  if (err != ESP_OK) {
    esp_err_t cleanup_err = s_tests[index].stop(&s_runtime);
    if (cleanup_err != ESP_OK) {
      s_cleanup_blocked = true;
      printf("[CTRL] boot=%08" PRIx32 " run=%" PRIu32
             " startup_cleanup=ERROR:%s code=%d; mode switching blocked\r\n",
             s_runtime.boot_id, s_runtime.run_id,
             esp_err_to_name(cleanup_err), (int)cleanup_err);
    }
  }
  s_test_active = err == ESP_OK;
  diag_print_test_header(&s_runtime, &s_tests[index],
                         err == ESP_OK
                             ? (s_runtime.init_status ? s_runtime.init_status : "OK")
                             : "ERROR",
                         index == 3 ? (s_runtime.config.t3_use_speed_sensor_driver
                                            ? "GENERATOR_DRIVER_CAPTURE"
                                            : "GENERATOR_DIAGNOSTIC_CAPTURE") :
                         index == 5 ? "SCOPE_ONLY" :
                         index == 6 ? "DRIVER_EDGE_CAPTURE" : "GPIO_DIAGNOSTIC");
  if (err != ESP_OK)
    printf("[%s] boot=%08" PRIx32 " run=%" PRIu32
           " INIT_ERROR=%s code=%d\r\n",
           s_tests[index].id, s_runtime.boot_id, s_runtime.run_id,
           esp_err_to_name(err), (int)err);
  else if (s_runtime.init_status)
    printf("[%s] boot=%08" PRIx32 " run=%" PRIu32
           " INIT_PARTIAL=%s\r\n",
           s_tests[index].id, s_runtime.boot_id, s_runtime.run_id,
           s_runtime.init_status);
  else
    printf("[%s] boot=%08" PRIx32 " run=%" PRIu32 " INIT_OK\r\n",
           s_tests[index].id, s_runtime.boot_id, s_runtime.run_id);
  if (err == ESP_OK && s_runtime.init_error != ESP_OK)
    printf("[%s] boot=%08" PRIx32 " run=%" PRIu32
           " INIT_DETAIL=PCNT_UNAVAILABLE error=%s code=%d; GPIO_ISR_CAPTURE_ACTIVE=1\r\n",
           s_tests[index].id, s_runtime.boot_id, s_runtime.run_id,
           esp_err_to_name(s_runtime.init_error), (int)s_runtime.init_error);
  return err;
}

static char *skip_space(char *text) {
  while (*text && isspace((unsigned char)*text)) ++text;
  return text;
}

static bool parse_u32(const char *text, uint32_t *value) {
  if (!text || !*text || !value) return false;
  char *end = NULL;
  unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end || parsed > UINT32_MAX) return false;
  *value = (uint32_t)parsed;
  return true;
}

static void command_error(const char *command, esp_err_t err) {
  printf("[CMD] boot=%08" PRIx32 " run=%" PRIu32
         " command=\"%s\" ERROR=%s code=%d\r\n",
         s_runtime.boot_id, s_runtime.run_id, command,
         esp_err_to_name(err), (int)err);
}

static void process_command(char *line) {
  char *cursor = skip_space(line);
  char *save = NULL;
  char *verb = strtok_r(cursor, " \t", &save);
  if (!verb) return;

  if ((verb[0] == 'T' || verb[0] == 't') && verb[1] >= '0' &&
      verb[1] <= '7' && verb[2] == '\0') {
    unsigned index = (unsigned)(verb[1] - '0');
    (void)start_test(index);
    return;
  }
  if (!strcasecmp(verb, "HELP")) {
    print_help();
    return;
  }
  if (!strcasecmp(verb, "EDGE")) {
    char *value = strtok_r(NULL, " \t", &save);
    if (!value) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    diag_edge_mode_t edge;
    if (!strcasecmp(value, "RISING")) edge = DIAG_EDGE_RISING;
    else if (!strcasecmp(value, "FALLING")) edge = DIAG_EDGE_FALLING;
    else if (!strcasecmp(value, "BOTH")) edge = DIAG_EDGE_BOTH;
    else {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    if ((s_test_index == 6 || s_test_index == 7) && edge == DIAG_EDGE_BOTH) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    s_runtime.config.edge_mode = edge;
    if (s_test_index == 2 || s_test_index == 6 || s_test_index == 7)
      (void)start_test(s_test_index);
    else
      printf("[CMD] boot=%08" PRIx32 " edge_saved=%s (applies to T2/T6/T7)\r\n",
             s_runtime.boot_id, diag_edge_name(edge));
    return;
  }
  if (!strcasecmp(verb, "FREQ")) {
    char *value = strtok_r(NULL, " \t", &save);
    uint32_t hz;
    if (s_test_index != 3 || !parse_u32(value, &hz) ||
        (hz != 1 && hz != 5 && hz != 10 && hz != 20 && hz != 100 &&
         hz != 500 && hz != 1000)) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    s_runtime.config.generator_frequency_hz = hz;
    (void)start_test(3);
    return;
  }
  if (!strcasecmp(verb, "CAPTURE")) {
    char *value = strtok_r(NULL, " \t", &save);
    if (s_test_index != 3 || !value ||
        (strcasecmp(value, "DIAG") && strcasecmp(value, "DRIVER"))) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    s_runtime.config.t3_use_speed_sensor_driver = !strcasecmp(value, "DRIVER");
    (void)start_test(3);
    return;
  }
  if (!strcasecmp(verb, "FILTER")) {
    char *value = strtok_r(NULL, " \t", &save);
    uint32_t microseconds;
    if (s_test_index != 7 || !parse_u32(value, &microseconds) ||
        microseconds >= (uint64_t)s_runtime.config.timeout_ms * 1000U) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    esp_err_t err = test_t7_set_filter(&s_runtime, microseconds);
    if (err != ESP_OK) command_error(line, err);
    else printf("[CMD] boot=%08" PRIx32 " run=%" PRIu32
                " FILTER_MIN_INTERVAL_US=%" PRIu32 " (scope review required)\r\n",
                s_runtime.boot_id, s_runtime.run_id, microseconds);
    return;
  }
  if (!strcasecmp(verb, "PPR")) {
    char *value = strtok_r(NULL, " \t", &save);
    uint32_t ppr;
    if (s_test_index != 7 || !parse_u32(value, &ppr) || ppr == 0 ||
        ppr > 1000000) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    s_runtime.config.pulses_per_revolution = ppr;
    s_runtime.config.rpm_relationship_confirmed = false;
    printf("[CMD] boot=%08" PRIx32 " run=%" PRIu32
           " PPR=%" PRIu32 " RPM_CONFIRMATION_CLEARED=1\r\n",
           s_runtime.boot_id, s_runtime.run_id, ppr);
    return;
  }
  if (!strcasecmp(verb, "RPM")) {
    char *value = strtok_r(NULL, " \t", &save);
    if (s_test_index != 7 || !value ||
        (strcasecmp(value, "CONFIRM") && strcasecmp(value, "OFF"))) {
      command_error(line, ESP_ERR_INVALID_ARG);
      return;
    }
    s_runtime.config.rpm_relationship_confirmed = !strcasecmp(value, "CONFIRM");
    printf("[CMD] boot=%08" PRIx32 " run=%" PRIu32
           " RPM_RELATIONSHIP=%s (operator declaration; verify independent reference)\r\n",
           s_runtime.boot_id, s_runtime.run_id,
           s_runtime.config.rpm_relationship_confirmed ? "CONFIRMED" : "UNVERIFIED");
    return;
  }
  if (!strcasecmp(verb, "MARK")) {
    char *value = strtok_r(NULL, " \t", &save);
    esp_err_t err = s_test_index == 4 ? test_t4_mark(&s_runtime, value)
                                     : ESP_ERR_INVALID_STATE;
    if (err != ESP_OK) command_error(line, err);
    return;
  }
  if (!strcasecmp(verb, "NOTE")) {
    char *value = save ? skip_space(save) : NULL;
    esp_err_t err = s_test_index == 5 && value && *value
                        ? test_t5_note(&s_runtime, value)
                        : ESP_ERR_INVALID_STATE;
    if (err != ESP_OK) command_error(line, err);
    return;
  }
  command_error(line, ESP_ERR_NOT_SUPPORTED);
}

void app_main(void) {
  s_runtime.boot_id = esp_random();
  s_runtime.run_id = 0;
  s_runtime.boot_started_us = esp_timer_get_time();
  s_runtime.config = diag_config_from_menuconfig();
  s_command_queue = xQueueCreate(4, sizeof(command_t));
  diag_print_boot_report(s_runtime.boot_id);
  printf("[CTRL] Serial commands available; enter HELP. Console baud=115200\r\n");
  if (!s_command_queue) {
    printf("[BOOT] ERROR command queue allocation failed; starting selected test only\r\n");
  } else {
    BaseType_t task = xTaskCreate(serial_input_task, "diag_serial_in", 4096,
                                  NULL, 4, NULL);
    if (task != pdPASS)
      printf("[BOOT] ERROR serial input task creation failed; mode switching unavailable\r\n");
  }

  (void)start_test(CONFIG_DIAG_START_TEST);
  TickType_t interval = pdMS_TO_TICKS(s_runtime.config.sample_interval_ms);
  if (interval < 1) interval = 1;
  for (;;) {
    command_t command;
    while (s_command_queue && xQueueReceive(s_command_queue, &command, 0) == pdTRUE)
      process_command(command.text);
    /* This is a task-dispatch timestamp for heartbeat modes. Capture modes
     * obtain their read reference after copying ISR state; this value must not
     * be used to timestamp a snapshot. */
    int64_t dispatch_time_us = esp_timer_get_time();
    if (s_test_active) {
      esp_err_t err = s_tests[s_test_index].update(&s_runtime,
                                                  dispatch_time_us);
      if (err != ESP_OK) {
        printf("[%s] boot=%08" PRIx32 " run=%" PRIu32
               " UPDATE_ERROR=%s code=%d\r\n",
               s_tests[s_test_index].id, s_runtime.boot_id, s_runtime.run_id,
               esp_err_to_name(err), (int)err);
        (void)stop_current_test();
      }
    }
    vTaskDelay(interval);
  }
}
