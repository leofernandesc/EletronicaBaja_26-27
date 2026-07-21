#include "sd_protocol_types.h"

esp_err_t datalogger_init(sdmmc_card_t *card, const char *mount_point,
                          sdmmc_host_t *host);
esp_err_t datalogger_deinit(const char *mount_point, sdmmc_card_t *card,
                            const sdmmc_host_t *host);
esp_err_t datalogger_append_to_file(const char *path, char *data);
