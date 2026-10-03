/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gps.h"
#include "nmea_frame.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief NMEA Parser runtime buffer size
 *
 */
#define NMEA_PARSER_RUNTIME_BUFFER_SIZE                                        \
  (CONFIG_NMEA_PARSER_RING_BUFFER_SIZE / 2)
#define NMEA_MAX_STATEMENT_ITEM_LENGTH (NMEA_MAX_FIELD_LENGTH + 1)
#define NMEA_EVENT_LOOP_QUEUE_SIZE (16)

#if !CONFIG_NMEA_STATEMENT_RMC
#error "RMC must be enabled because it defines the GPS logging cadence"
#endif

/**
 * @brief Define of NMEA Parser Event base
 *
 */
ESP_EVENT_DEFINE_BASE(ESP_NMEA_EVENT);

static const char *GPS_TAG = "nmea_parser";

/**
 * @brief GPS parser library runtime structure
 *
 */
typedef struct {
  uint8_t item_num;         /*!< Current item number */
  uint8_t sat_num;          /*!< Satellite number */
  uint8_t sat_count;        /*!< Satellite count */
  uint8_t cur_statement;    /*!< Current statement ID */
  char item_str[NMEA_MAX_STATEMENT_ITEM_LENGTH]; /*!< Current item */
  gps_t parent;                                  /*!< Parent class */
  uart_port_t uart_port;                         /*!< Uart port number */
  uint8_t *buffer;                               /*!< Runtime buffer */
  esp_event_loop_handle_t event_loop_hdl;        /*!< Event loop handle */
  TaskHandle_t tsk_hdl;                          /*!< NMEA Parser task handle */
  QueueHandle_t event_queue;                     /*!< UART event queue handle */
  gps_t last_gga;
  int64_t last_gga_us;
  bool has_gga;
  uint32_t dropped_events;
  uint32_t input_errors;
} esp_gps_t;

/**
 * @brief parse latitude or longitude
 *              format of latitude in NMEA is ddmm.sss and longitude is
 * dddmm.sss
 * @param esp_gps esp_gps_t type object
 * @return float Latitude or Longitude value (unit: degree)
 */
static float parse_lat_long(esp_gps_t *esp_gps) {
  float ll = strtof(esp_gps->item_str, NULL);
  int deg = ((int)ll) / 100;
  float min = ll - (deg * 100);
  ll = deg + min / 60.0f;
  return ll;
}

/**
 * @brief Converter two continuous numeric character into a uint8_t number
 *
 * @param digit_char numeric character
 * @return uint8_t result of converting
 */
static inline uint8_t convert_two_digit2number(const char *digit_char) {
  return 10 * (digit_char[0] - '0') + (digit_char[1] - '0');
}

/**
 * @brief Parse UTC time in GPS statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_utc_time(esp_gps_t *esp_gps) {
  if (strlen(esp_gps->item_str) < 6) return;
  for (int i = 0; i < 6; i++) {
    if (!isdigit((unsigned char)esp_gps->item_str[i])) return;
  }
  esp_gps->parent.tim.hour = convert_two_digit2number(esp_gps->item_str + 0);
  esp_gps->parent.tim.minute = convert_two_digit2number(esp_gps->item_str + 2);
  esp_gps->parent.tim.second = convert_two_digit2number(esp_gps->item_str + 4);
  if (esp_gps->parent.tim.hour >= 24 || esp_gps->parent.tim.minute >= 60 ||
      esp_gps->parent.tim.second >= 60) return;
  if (esp_gps->item_str[6] == '.') {
    uint16_t tmp = 0;
    uint8_t i = 7;
    while (esp_gps->item_str[i] && i < 10) {
      if (!isdigit((unsigned char)esp_gps->item_str[i])) return;
      tmp = 10 * tmp + esp_gps->item_str[i] - '0';
      i++;
    }
    while (i++ < 10) tmp *= 10;
    esp_gps->parent.tim.thousand = tmp;
  }
  esp_gps->parent.time_valid = true;
}

#if CONFIG_NMEA_STATEMENT_GGA
/**
 * @brief Parse GGA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gga(esp_gps_t *esp_gps) {
  /* Process GGA statement */
  switch (esp_gps->item_num) {
  case 1: /* Process UTC time */
    parse_utc_time(esp_gps);
    break;
  case 2: /* Latitude */
    esp_gps->parent.latitude = parse_lat_long(esp_gps);
    break;
  case 3: /* Latitude north(1)/south(-1) information */
    if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
      esp_gps->parent.latitude *= -1;
    }
    break;
  case 4: /* Longitude */
    esp_gps->parent.longitude = parse_lat_long(esp_gps);
    break;
  case 5: /* Longitude east(1)/west(-1) information */
    if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
      esp_gps->parent.longitude *= -1;
    }
    break;
  case 6: /* Fix status */
    esp_gps->parent.fix = (gps_fix_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  case 7: /* Satellites in use */
    esp_gps->parent.sats_in_use = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  case 8: /* HDOP */
    esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
    break;
  case 9: /* Altitude */
    esp_gps->parent.altitude = strtof(esp_gps->item_str, NULL);
    break;
  case 11: /* Geoid separation */
    // Keep field 9: altitude above mean sea level, in meters.
    break;
  default:
    break;
  }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSA
/**
 * @brief Parse GSA statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsa(esp_gps_t *esp_gps) {
  /* Process GSA statement */
  switch (esp_gps->item_num) {
  case 2: /* Process fix mode */
    esp_gps->parent.fix_mode =
        (gps_fix_mode_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  case 15: /* Process PDOP */
    esp_gps->parent.dop_p = strtof(esp_gps->item_str, NULL);
    break;
  case 16: /* Process HDOP */
    esp_gps->parent.dop_h = strtof(esp_gps->item_str, NULL);
    break;
  case 17: /* Process VDOP */
    esp_gps->parent.dop_v = strtof(esp_gps->item_str, NULL);
    break;
  default:
    /* Parse satellite IDs */
    if (esp_gps->item_num >= 3 && esp_gps->item_num <= 14) {
      esp_gps->parent.sats_id_in_use[esp_gps->item_num - 3] =
          (uint8_t)strtol(esp_gps->item_str, NULL, 10);
    }
    break;
  }
}
#endif

#if CONFIG_NMEA_STATEMENT_GSV
/**
 * @brief Parse GSV statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gsv(esp_gps_t *esp_gps) {
  /* Process GSV statement */
  switch (esp_gps->item_num) {
  case 1: /* total GSV numbers */
    esp_gps->sat_count = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  case 2: /* Current GSV statement number */
    esp_gps->sat_num = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  case 3: /* Process satellites in view */
    esp_gps->parent.sats_in_view = (uint8_t)strtol(esp_gps->item_str, NULL, 10);
    break;
  default:
    if (esp_gps->item_num >= 4 && esp_gps->item_num <= 19) {
      uint8_t item_num =
          esp_gps->item_num - 4; /* Normalize item number from 4-19 to 0-15 */
      uint8_t index;
      uint32_t value;
      index = 4 * (esp_gps->sat_num - 1) + item_num / 4; /* Get array index */
      if (index < GPS_MAX_SATELLITES_IN_VIEW) {
        value = strtol(esp_gps->item_str, NULL, 10);
        switch (item_num % 4) {
        case 0:
          esp_gps->parent.sats_desc_in_view[index].num = (uint8_t)value;
          break;
        case 1:
          esp_gps->parent.sats_desc_in_view[index].elevation = (uint8_t)value;
          break;
        case 2:
          esp_gps->parent.sats_desc_in_view[index].azimuth = (uint16_t)value;
          break;
        case 3:
          esp_gps->parent.sats_desc_in_view[index].snr = (uint8_t)value;
          break;
        default:
          break;
        }
      }
    }
    break;
  }
}
#endif

#if CONFIG_NMEA_STATEMENT_RMC
/**
 * @brief Parse RMC statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_rmc(esp_gps_t *esp_gps) {
  /* Process GPRMC statement */
  switch (esp_gps->item_num) {
  case 1: /* Process UTC time */
    parse_utc_time(esp_gps);
    break;
  case 2: /* Process valid status */
    esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
    break;
  case 3: /* Latitude */
    esp_gps->parent.latitude = parse_lat_long(esp_gps);
    break;
  case 4: /* Latitude north(1)/south(-1) information */
    if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
      esp_gps->parent.latitude *= -1;
    }
    break;
  case 5: /* Longitude */
    esp_gps->parent.longitude = parse_lat_long(esp_gps);
    break;
  case 6: /* Longitude east(1)/west(-1) information */
    if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
      esp_gps->parent.longitude *= -1;
    }
    break;
  case 7: /* Process ground speed in unit m/s */
    esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) * 0.514444;
    break;
  case 8: /* Process true course over ground */
    esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
    break;
  case 9: /* Process date */
    if (strlen(esp_gps->item_str) == 6) {
      bool digits = true;
      for (int i = 0; i < 6; i++)
        digits &= isdigit((unsigned char)esp_gps->item_str[i]) != 0;
      if (digits) {
        esp_gps->parent.date.day = convert_two_digit2number(esp_gps->item_str);
        esp_gps->parent.date.month = convert_two_digit2number(esp_gps->item_str + 2);
        esp_gps->parent.date.year = convert_two_digit2number(esp_gps->item_str + 4);
        esp_gps->parent.date_valid = esp_gps->parent.date.day >= 1 &&
            esp_gps->parent.date.day <= 31 && esp_gps->parent.date.month >= 1 &&
            esp_gps->parent.date.month <= 12;
      }
    }
    break;
  case 10: /* Process magnetic variation */
    esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
    break;
  default:
    break;
  }
}
#endif

#if CONFIG_NMEA_STATEMENT_GLL
/**
 * @brief Parse GLL statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_gll(esp_gps_t *esp_gps) {
  /* Process GPGLL statement */
  switch (esp_gps->item_num) {
  case 1: /* Latitude */
    esp_gps->parent.latitude = parse_lat_long(esp_gps);
    break;
  case 2: /* Latitude north(1)/south(-1) information */
    if (esp_gps->item_str[0] == 'S' || esp_gps->item_str[0] == 's') {
      esp_gps->parent.latitude *= -1;
    }
    break;
  case 3: /* Longitude */
    esp_gps->parent.longitude = parse_lat_long(esp_gps);
    break;
  case 4: /* Longitude east(1)/west(-1) information */
    if (esp_gps->item_str[0] == 'W' || esp_gps->item_str[0] == 'w') {
      esp_gps->parent.longitude *= -1;
    }
    break;
  case 5: /* Process UTC time */
    parse_utc_time(esp_gps);
    break;
  case 6: /* Process valid status */
    esp_gps->parent.valid = (esp_gps->item_str[0] == 'A');
    break;
  default:
    break;
  }
}
#endif

#if CONFIG_NMEA_STATEMENT_VTG
/**
 * @brief Parse VTG statements
 *
 * @param esp_gps esp_gps_t type object
 */
static void parse_vtg(esp_gps_t *esp_gps) {
  /* Process GPVGT statement */
  switch (esp_gps->item_num) {
  case 1: /* Process true course over ground */
    esp_gps->parent.cog = strtof(esp_gps->item_str, NULL);
    break;
  case 3: /* Process magnetic variation */
    esp_gps->parent.variation = strtof(esp_gps->item_str, NULL);
    break;
  case 5: /* Process ground speed in unit m/s */
    esp_gps->parent.speed =
        strtof(esp_gps->item_str, NULL) * 0.514444; // knots to m/s
    break;
  case 7: /* Process ground speed in unit m/s */
    esp_gps->parent.speed = strtof(esp_gps->item_str, NULL) / 3.6; // km/h to
                                                                   // m/s
    break;
  default:
    break;
  }
}
#endif

/**
 * @brief Parse received item
 *
 * @param esp_gps esp_gps_t type object
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t parse_item(esp_gps_t *esp_gps) {
  esp_err_t err = ESP_OK;
  /* start of a statement */
  if (esp_gps->item_num == 0 && esp_gps->item_str[0] == '$') {
    if (0) {
    }
#if CONFIG_NMEA_STATEMENT_GGA
    else if (strstr(esp_gps->item_str, "GGA")) {
      esp_gps->cur_statement = STATEMENT_GGA;
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
    else if (strstr(esp_gps->item_str, "GSA")) {
      esp_gps->cur_statement = STATEMENT_GSA;
    }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
    else if (strstr(esp_gps->item_str, "RMC")) {
      esp_gps->cur_statement = STATEMENT_RMC;
    }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
    else if (strstr(esp_gps->item_str, "GSV")) {
      esp_gps->cur_statement = STATEMENT_GSV;
    }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
    else if (strstr(esp_gps->item_str, "GLL")) {
      esp_gps->cur_statement = STATEMENT_GLL;
    }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
    else if (strstr(esp_gps->item_str, "VTG")) {
      esp_gps->cur_statement = STATEMENT_VTG;
    }
#endif
    else {
      esp_gps->cur_statement = STATEMENT_UNKNOWN;
    }
    goto out;
  }
  /* Parse each item, depend on the type of the statement */
  if (esp_gps->cur_statement == STATEMENT_UNKNOWN) {
    goto out;
  }
#if CONFIG_NMEA_STATEMENT_GGA
  else if (esp_gps->cur_statement == STATEMENT_GGA) {
    parse_gga(esp_gps);
  }
#endif
#if CONFIG_NMEA_STATEMENT_GSA
  else if (esp_gps->cur_statement == STATEMENT_GSA) {
    parse_gsa(esp_gps);
  }
#endif
#if CONFIG_NMEA_STATEMENT_GSV
  else if (esp_gps->cur_statement == STATEMENT_GSV) {
    parse_gsv(esp_gps);
  }
#endif
#if CONFIG_NMEA_STATEMENT_RMC
  else if (esp_gps->cur_statement == STATEMENT_RMC) {
    parse_rmc(esp_gps);
  }
#endif
#if CONFIG_NMEA_STATEMENT_GLL
  else if (esp_gps->cur_statement == STATEMENT_GLL) {
    parse_gll(esp_gps);
  }
#endif
#if CONFIG_NMEA_STATEMENT_VTG
  else if (esp_gps->cur_statement == STATEMENT_VTG) {
    parse_vtg(esp_gps);
  }
#endif
  else {
    err = ESP_FAIL;
  }
out:
  return err;
}

/**
 * @brief Parse NMEA statements from GPS receiver
 *
 * @param esp_gps esp_gps_t type object
 * @param len number of bytes to decode
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
static bool same_utc_second(const gps_time_t *a, const gps_time_t *b) {
  return a->hour == b->hour && a->minute == b->minute &&
         a->second == b->second;
}

static bool number_valid(nmea_field_t field) {
  if (field.length == 0) return false;
  char text[NMEA_MAX_STATEMENT_ITEM_LENGTH];
  memcpy(text, field.start, field.length);
  text[field.length] = '\0';
  char *end = NULL;
  float value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

static bool coordinate_valid(nmea_field_t value, nmea_field_t hemisphere,
                             float max_degrees, char positive, char negative) {
  int direction = hemisphere.length == 1
      ? toupper((unsigned char)hemisphere.start[0]) : 0;
  if (value.length == 0 || hemisphere.length != 1 ||
      (direction != positive && direction != negative))
    return false;
  char text[NMEA_MAX_STATEMENT_ITEM_LENGTH];
  memcpy(text, value.start, value.length);
  text[value.length] = '\0';
  char *end = NULL;
  float raw = strtof(text, &end);
  if (end == text || *end != '\0' || !isfinite(raw) || raw < 0) return false;
  float degrees = floorf(raw / 100.0f);
  float minutes = raw - degrees * 100.0f;
  return minutes >= 0 && minutes < 60 &&
         degrees + minutes / 60.0f <= max_degrees;
}

static esp_err_t gps_decode(esp_gps_t *esp_gps, size_t len) {
  nmea_field_t fields[NMEA_MAX_FIELDS];
  size_t field_count = 0;
  if (nmea_frame_split(esp_gps->buffer, len, fields, &field_count) != 0) {
    __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
    return ESP_FAIL;
  }

  // A sentence must be complete and checksummed before it may change gps_t.
  esp_gps->cur_statement = STATEMENT_UNKNOWN;
  esp_gps->item_num = 0;
  memset(esp_gps->item_str, 0, sizeof(esp_gps->item_str));
  memcpy(esp_gps->item_str, fields[0].start, fields[0].length);
  parse_item(esp_gps);

  if (esp_gps->cur_statement == STATEMENT_GGA ||
      esp_gps->cur_statement == STATEMENT_RMC) {
    memset(&esp_gps->parent, 0, sizeof(esp_gps->parent));
  }
  for (size_t i = 1; i < field_count; i++) {
    esp_gps->item_num = (uint8_t)i;
    memset(esp_gps->item_str, 0, sizeof(esp_gps->item_str));
    memcpy(esp_gps->item_str, fields[i].start, fields[i].length);
    parse_item(esp_gps);
  }

  if (esp_gps->cur_statement == STATEMENT_GGA) {
    esp_gps->last_gga = esp_gps->parent;
    esp_gps->last_gga.altitude_valid =
        field_count > 9 && number_valid(fields[9]);
    esp_gps->last_gga_us = esp_timer_get_time();
    esp_gps->has_gga = true;
  } else if (esp_gps->cur_statement == STATEMENT_RMC) {
    // RMC supplies the date, position and validity for each emitted fix.
    // GGA altitude is optional and must refer to the same UTC second.
    bool coordinates_present = field_count > 9 &&
        coordinate_valid(fields[3], fields[4], 90.0f, 'N', 'S') &&
        coordinate_valid(fields[5], fields[6], 180.0f, 'E', 'W');
    if (!coordinates_present || !esp_gps->parent.time_valid ||
        !esp_gps->parent.date_valid) esp_gps->parent.valid = false;
    esp_gps->parent.speed_valid =
        field_count > 7 && number_valid(fields[7]);
    if (esp_gps->parent.valid && esp_gps->has_gga &&
        esp_gps->last_gga.fix != GPS_FIX_INVALID &&
        esp_gps->last_gga.time_valid &&
        same_utc_second(&esp_gps->parent.tim, &esp_gps->last_gga.tim) &&
        esp_timer_get_time() - esp_gps->last_gga_us <= 2000000) {
      esp_gps->parent.altitude = esp_gps->last_gga.altitude;
      esp_gps->parent.altitude_valid = esp_gps->last_gga.altitude_valid;
    }
    esp_err_t err = esp_event_post_to(
        esp_gps->event_loop_hdl, ESP_NMEA_EVENT, GPS_UPDATE,
        &esp_gps->parent, sizeof(gps_t), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
      uint32_t count = __atomic_add_fetch(&esp_gps->dropped_events, 1,
                                           __ATOMIC_RELAXED);
      if (count == 1 || count % 100 == 0)
        ESP_LOGW(GPS_TAG, "GPS events dropped: %lu (%s)",
                 (unsigned long)count, esp_err_to_name(err));
    }
  }
  return ESP_OK;
}

/**
 * @brief Handle when a pattern has been detected by uart
 *
 * @param esp_gps esp_gps_t type object
 */
static void esp_handle_uart_pattern(esp_gps_t *esp_gps) {
  int pos = uart_pattern_pop_pos(esp_gps->uart_port);
  if (pos != -1) {
    if ((size_t)pos + 1 >= NMEA_PARSER_RUNTIME_BUFFER_SIZE) {
      ESP_LOGW(GPS_TAG, "NMEA line exceeds parser buffer");
      __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
      uart_flush_input(esp_gps->uart_port);
      return;
    }
    /* read one line(include '\n') */
    int read_len = uart_read_bytes(esp_gps->uart_port, esp_gps->buffer, pos + 1,
                                   100 / portTICK_PERIOD_MS);
    if (read_len != pos + 1) {
      __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
      uart_flush_input(esp_gps->uart_port);
      return;
    }
    /* make sure the line is a standard string */
    esp_gps->buffer[read_len] = '\0';
    /* Send new line to handle */
    if (gps_decode(esp_gps, read_len) != ESP_OK) {
      uint32_t count = __atomic_load_n(&esp_gps->input_errors,
                                        __ATOMIC_RELAXED);
      if (count == 1 || count % 100 == 0)
        ESP_LOGW(GPS_TAG, "GPS input errors: %lu", (unsigned long)count);
    }
  } else {
    ESP_LOGW(GPS_TAG, "Pattern Queue Size too small");
    __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
    uart_flush_input(esp_gps->uart_port);
  }
}

/**
 * @brief NMEA Parser Task Entry
 *
 * @param arg argument
 */
static void nmea_parser_task_entry(void *arg) {
  esp_gps_t *esp_gps = (esp_gps_t *)arg;
  uart_event_t event;
  while (1) {
    if (xQueueReceive(esp_gps->event_queue, &event, pdMS_TO_TICKS(200))) {
      switch (event.type) {
      case UART_DATA:
        break;
      case UART_FIFO_OVF:
        ESP_LOGW(GPS_TAG, "HW FIFO Overflow");
        __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
        uart_flush(esp_gps->uart_port);
        xQueueReset(esp_gps->event_queue);
        break;
      case UART_BUFFER_FULL:
        ESP_LOGW(GPS_TAG, "Ring Buffer Full");
        __atomic_fetch_add(&esp_gps->input_errors, 1, __ATOMIC_RELAXED);
        uart_flush(esp_gps->uart_port);
        xQueueReset(esp_gps->event_queue);
        break;
      case UART_BREAK:
        ESP_LOGW(GPS_TAG, "Rx Break");
        break;
      case UART_PARITY_ERR:
        ESP_LOGE(GPS_TAG, "Parity Error");
        break;
      case UART_FRAME_ERR:
        ESP_LOGE(GPS_TAG, "Frame Error");
        break;
      case UART_PATTERN_DET:
        esp_handle_uart_pattern(esp_gps);
        break;
      default:
        ESP_LOGW(GPS_TAG, "unknown uart event type: %d", event.type);
        break;
      }
    }
    /* Drive the event loop */
    esp_event_loop_run(esp_gps->event_loop_hdl, pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}

/**
 * @brief Init NMEA Parser
 *
 * @param config Configuration of NMEA Parser
 * @return nmea_parser_handle_t handle of nmea_parser
 */
nmea_parser_handle_t nmea_parser_init(const nmea_parser_config_t *config) {
  esp_gps_t *esp_gps = calloc(1, sizeof(esp_gps_t));
  if (!esp_gps) {
    ESP_LOGE(GPS_TAG, "calloc memory for esp_fps failed");
    goto err_gps;
  }
  esp_gps->buffer = calloc(1, NMEA_PARSER_RUNTIME_BUFFER_SIZE);
  if (!esp_gps->buffer) {
    ESP_LOGE(GPS_TAG, "calloc memory for runtime buffer failed");
    goto err_buffer;
  }
  esp_gps->uart_port = config->uart.uart_port;
  /* Install UART friver */
  uart_config_t uart_config = {
      .baud_rate = config->uart.baud_rate,
      .data_bits = config->uart.data_bits,
      .parity = config->uart.parity,
      .stop_bits = config->uart.stop_bits,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  if (uart_driver_install(
          esp_gps->uart_port, CONFIG_NMEA_PARSER_RING_BUFFER_SIZE, 0,
          config->uart.event_queue_size, &esp_gps->event_queue, 0) != ESP_OK) {
    ESP_LOGE(GPS_TAG, "install uart driver failed");
    goto err_buffer;
  }
  if (uart_param_config(esp_gps->uart_port, &uart_config) != ESP_OK) {
    ESP_LOGE(GPS_TAG, "config uart parameter failed");
    goto err_eloop;
  }
  if (uart_set_pin(esp_gps->uart_port, UART_PIN_NO_CHANGE, config->uart.rx_pin,
                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    ESP_LOGE(GPS_TAG, "config uart gpio failed");
    goto err_eloop;
  }
  /* Set pattern interrupt, used to detect the end of a line */
  if (uart_enable_pattern_det_baud_intr(esp_gps->uart_port, '\n', 1, 9, 0, 0)
      != ESP_OK) goto err_eloop;
  /* Set pattern queue size */
  if (uart_pattern_queue_reset(esp_gps->uart_port,
                               config->uart.event_queue_size) != ESP_OK)
    goto err_eloop;
  uart_flush(esp_gps->uart_port);
  /* Create Event loop */
  esp_event_loop_args_t loop_args = {.queue_size = NMEA_EVENT_LOOP_QUEUE_SIZE,
                                     .task_name = NULL};
  if (esp_event_loop_create(&loop_args, &esp_gps->event_loop_hdl) != ESP_OK) {
    ESP_LOGE(GPS_TAG, "create event loop failed");
    goto err_eloop;
  }
  /* Create NMEA Parser task */
  BaseType_t err = xTaskCreate(
      nmea_parser_task_entry, "nmea_parser", CONFIG_NMEA_PARSER_TASK_STACK_SIZE,
      esp_gps, CONFIG_NMEA_PARSER_TASK_PRIORITY, &esp_gps->tsk_hdl);
  if (err != pdTRUE) {
    ESP_LOGE(GPS_TAG, "create NMEA Parser task failed");
    goto err_task_create;
  }
  ESP_LOGI(GPS_TAG, "NMEA Parser init OK");
  return esp_gps;
  /*Error Handling*/
err_task_create:
  esp_event_loop_delete(esp_gps->event_loop_hdl);
err_eloop:
  uart_driver_delete(esp_gps->uart_port);
err_buffer:
  free(esp_gps->buffer);
err_gps:
  free(esp_gps);
  return NULL;
}

void nmea_parser_get_stats(nmea_parser_handle_t nmea_hdl,
                           nmea_parser_stats_t *stats) {
  if (!nmea_hdl || !stats) return;
  esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
  stats->dropped_events = __atomic_load_n(&esp_gps->dropped_events, __ATOMIC_RELAXED);
  stats->input_errors = __atomic_load_n(&esp_gps->input_errors, __ATOMIC_RELAXED);
}

/**
 * @brief Deinit NMEA Parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @return esp_err_t ESP_OK on success,ESP_FAIL on error
 */
esp_err_t nmea_parser_deinit(nmea_parser_handle_t nmea_hdl) {
  esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
  vTaskDelete(esp_gps->tsk_hdl);
  esp_event_loop_delete(esp_gps->event_loop_hdl);
  esp_err_t err = uart_driver_delete(esp_gps->uart_port);
  free(esp_gps->buffer);
  free(esp_gps);
  return err;
}

/**
 * @brief Add user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_NO_MEM: Cannot allocate memory for the handler
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t nmea_parser_add_handler(nmea_parser_handle_t nmea_hdl,
                                  esp_event_handler_t event_handler,
                                  void *handler_args) {
  esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
  return esp_event_handler_register_with(esp_gps->event_loop_hdl,
                                         ESP_NMEA_EVENT, ESP_EVENT_ANY_ID,
                                         event_handler, handler_args);
}

/**
 * @brief Remove user defined handler for NMEA parser
 *
 * @param nmea_hdl handle of NMEA parser
 * @param event_handler user defined event handler
 * @return esp_err_t
 *  - ESP_OK: Success
 *  - ESP_ERR_INVALIG_ARG: Invalid combination of event base and event id
 *  - Others: Fail
 */
esp_err_t nmea_parser_remove_handler(nmea_parser_handle_t nmea_hdl,
                                     esp_event_handler_t event_handler) {
  esp_gps_t *esp_gps = (esp_gps_t *)nmea_hdl;
  return esp_event_handler_unregister_with(
      esp_gps->event_loop_hdl, ESP_NMEA_EVENT, ESP_EVENT_ANY_ID, event_handler);
}
