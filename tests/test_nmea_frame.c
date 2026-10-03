#include "nmea_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static size_t sentence(char *out, size_t capacity, const char *body) {
  unsigned checksum = 0;
  for (const char *p = body; *p; p++) checksum ^= (unsigned char)*p;
  int length = snprintf(out, capacity, "$%s*%02X\r\n", body, checksum);
  assert(length > 0 && (size_t)length < capacity);
  return (size_t)length;
}

int main(void) {
  char line[256];
  nmea_field_t fields[NMEA_MAX_FIELDS];
  size_t count = 0;
  size_t length = sentence(line, sizeof(line),
      "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,,");
  assert(nmea_frame_split((const uint8_t *)line, length, fields, &count) == 0);
  assert(count == 12);
  assert(fields[2].length == 1 && fields[2].start[0] == 'A');

  line[7] = '9';
  assert(nmea_frame_split((const uint8_t *)line, length, fields, &count) != 0);
  length = sentence(line, sizeof(line), "GPRMC,123519,V,,,,,,,230394,,");
  assert(nmea_frame_split((const uint8_t *)line, length, fields, &count) == 0);
  assert(fields[3].length == 0);

  length = sentence(line, sizeof(line),
      "GPRMC,1234567890123456,A,4807.038,N,01131.000,E");
  assert(nmea_frame_split((const uint8_t *)line, length, fields, &count) != 0);
  length = sentence(line, sizeof(line), "GPRMC,123519,V,,,,,,,230394,,");
  assert(nmea_frame_split((const uint8_t *)line, length - 1, fields, &count) != 0);
  line[length - 3] = 'X';
  assert(nmea_frame_split((const uint8_t *)line, length, fields, &count) != 0);

  // The UART can deliver arbitrary bytes. Framing must stay within the input.
  uint32_t seed = 0x12345678;
  for (int trial = 0; trial < 20000; trial++) {
    size_t input_length = (size_t)(trial % sizeof(line));
    for (size_t i = 0; i < input_length; i++) {
      seed = seed * 1664525u + 1013904223u;
      line[i] = (char)(seed >> 24);
    }
    int result = nmea_frame_split((const uint8_t *)line, input_length,
                                  fields, &count);
    if (result == 0) {
      assert(count <= NMEA_MAX_FIELDS);
      for (size_t i = 0; i < count; i++) {
        assert(fields[i].start >= line);
        assert(fields[i].start + fields[i].length <= line + input_length);
      }
    }
  }
  puts("NMEA frame tests passed");
}
