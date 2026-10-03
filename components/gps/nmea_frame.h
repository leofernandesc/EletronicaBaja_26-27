#pragma once

#include <stddef.h>
#include <stdint.h>

#define NMEA_MAX_FIELDS 32
#define NMEA_MAX_FIELD_LENGTH 15

typedef struct {
  const char *start;
  size_t length;
} nmea_field_t;

// Returns 0 only for a complete, checksummed, bounded NMEA line.
int nmea_frame_split(const uint8_t *line, size_t length,
                     nmea_field_t fields[NMEA_MAX_FIELDS], size_t *field_count);
