#include "datalogger.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sd_protocol_types.h"
#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "datalogger";

#define PIN_NUM_MISO CONFIG_PIN_MISO
#define PIN_NUM_MOSI CONFIG_PIN_MOSI
#define PIN_NUM_CLK CONFIG_PIN_CLK
#define PIN_NUM_CS CONFIG_PIN_CS

esp_err_t datalogger_init(sdmmc_card_t **card, sdmmc_host_t *host,
                          const char *mount_point) {
  esp_err_t ret;
  *card = NULL;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024};
  ESP_LOGI(TAG, "Inicializando cartão SD");

  ESP_LOGI(TAG, "Usando periféricos SPI");

  host->unaligned_multi_block_rw_max_chunk_size = 8;

  spi_bus_config_t bus_cfg = {
      .mosi_io_num = PIN_NUM_MOSI,
      .miso_io_num = PIN_NUM_MISO,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4000,
  };

  ret = spi_bus_initialize(host->slot, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao inicializar o barramento SPI.");
    return ret;
  }

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = host->slot;

  ESP_LOGI(TAG, "Montando o sistema de arquivos");
  ret = esp_vfs_fat_sdspi_mount(mount_point, host, &slot_config, &mount_config,
                                card);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Falha ao montar sistema de arquivos; dados preservados.");
    } else {
      ESP_LOGE(
          TAG,
          "Falha ao inicializar cartão (%s). "
          "Verifique se os pinos do cartão SD estão com os resistores pull-up.",
          esp_err_to_name(ret));
    }
    spi_bus_free(host->slot);
    *card = NULL;
    return ret;
  }
  ESP_LOGI(TAG, "Montagem do sistema de arquivos concluída");
  return ESP_OK;
}

esp_err_t datalogger_deinit(sdmmc_card_t **card, const sdmmc_host_t *host,
                            const char *mount_point) {
  if (!card || !*card) return ESP_ERR_INVALID_ARG;
  esp_err_t unmount_err = esp_vfs_fat_sdcard_unmount(mount_point, *card);
  // The mount helper may release card even when unregistering VFS fails.
  *card = NULL;
  esp_err_t bus_err = spi_bus_free(host->slot);
  if (unmount_err != ESP_OK)
    ESP_LOGE(TAG, "SD unmount failed: %s", esp_err_to_name(unmount_err));
  if (bus_err != ESP_OK)
    ESP_LOGE(TAG, "SPI bus release failed: %s", esp_err_to_name(bus_err));
  return unmount_err != ESP_OK ? unmount_err : bus_err;
}

esp_err_t datalogger_append_to_file(const char *path, const char *data) {
  ESP_LOGD(TAG, "Abrindo arquivo %s", path);
  FILE *f = fopen(path, "a");
  if (f == NULL) {
    ESP_LOGE(TAG, "Falha ao abrir arquivo para inserção");
    return ESP_FAIL;
  }
  bool ok = fputs(data, f) >= 0;
  if (ok) ok = fflush(f) == 0;
  if (ok) ok = fsync(fileno(f)) == 0;
  if (fclose(f) != 0) ok = false;
  if (!ok) {
    ESP_LOGE(TAG, "Falha ao gravar %s", path);
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "Inserção no arquivo concluída");

  return ESP_OK;
}

esp_err_t datalogger_prepare_file(const char *path, const char *header,
                                  bool *trimmed_tail) {
  *trimmed_tail = false;
  FILE *f = fopen(path, "r+");
  if (!f && errno == ENOENT) return datalogger_append_to_file(path, header);
  if (!f) return ESP_FAIL;
  if (fseek(f, 0, SEEK_END) != 0) goto fail;
  long size = ftell(f);
  if (size < 0) goto fail;
  if (size == 0) {
    if (fclose(f) != 0) return ESP_FAIL;
    return datalogger_append_to_file(path, header);
  }
  if (fseek(f, size - 1, SEEK_SET) != 0) goto fail;
  if (fgetc(f) != '\n') {
    long keep = size - 1;
    while (keep > 0) {
      if (fseek(f, keep - 1, SEEK_SET) != 0) goto fail;
      if (fgetc(f) == '\n') break;
      keep--;
    }
    if (ftruncate(fileno(f), keep) != 0) goto fail;
    if (fsync(fileno(f)) != 0) goto fail;
    *trimmed_tail = true;
    if (keep == 0) {
      if (fclose(f) != 0) return ESP_FAIL;
      return datalogger_append_to_file(path, header);
    }
  }
  return fclose(f) == 0 ? ESP_OK : ESP_FAIL;
fail:
  fclose(f);
  return ESP_FAIL;
}
