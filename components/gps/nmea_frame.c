#include "nmea_frame.h"

static int hex_digit(uint8_t value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

int nmea_frame_split(const uint8_t *line, size_t length,
                     nmea_field_t fields[NMEA_MAX_FIELDS], size_t *field_count) {
  if (!line || !fields || !field_count || length < 7 || line[0] != '$' ||
      line[length - 1] != '\n') return -1;

  size_t end = length - 1;
  if (line[end - 1] == '\r') end--;
  if (end < 4 || line[end - 3] != '*') return -1;
  int high = hex_digit(line[end - 2]);
  int low = hex_digit(line[end - 1]);
  if (high < 0 || low < 0) return -1;

  uint8_t checksum = 0;
  for (size_t i = 1; i < end - 3; i++) checksum ^= line[i];
  if (checksum != (uint8_t)((high << 4) | low)) return -1;

  size_t count = 0;
  size_t field_start = 0;
  for (size_t i = 0; i <= end - 3; i++) {
    if (i != end - 3 && line[i] != ',') continue;
    size_t field_length = i - field_start;
    if (count == NMEA_MAX_FIELDS || field_length > NMEA_MAX_FIELD_LENGTH)
      return -1;
    fields[count].start = (const char *)line + field_start;
    fields[count].length = field_length;
    count++;
    field_start = i + 1;
  }
  if (count == 0 || fields[0].length != 6) return -1;
  *field_count = count;
  return 0;
}
