#pragma once

#include "esp_err.h"
#include "sd_protocol_types.h"
#include "sdmmc_cmd.h"

esp_err_t datalogger_init(sdmmc_card_t **card, sdmmc_host_t *host,
                          const char *mount_point);
esp_err_t datalogger_deinit(sdmmc_card_t **card, const sdmmc_host_t *host,
                            const char *mount_point);
esp_err_t datalogger_append_to_file(const char *path, const char *data);
