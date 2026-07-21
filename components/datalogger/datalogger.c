#include "datalogger.h"
#include "driver/sdspi_common.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sd_protocol_types.h"
#include <stdio.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#define MAX_CHAR_SIZE 64

static const char *TAG = "datalogger";

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO CONFIG_PIN_MISO
#define PIN_NUM_MOSI CONFIG_PIN_MOSI
#define PIN_NUM_CLK CONFIG_PIN_CLK
#define PIN_NUM_CS CONFIG_PIN_CS

esp_err_t datalogger_init(sdmmc_card_t *card, const char *mount_point,
                          sdmmc_host_t *host) {
  esp_err_t ret;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_FORMAT_IF_MOUNT_FAILED
      .format_if_mount_failed = true,
#else
      .format_if_mount_failed = false,
#endif // FORMAT_IF_MOUNT_FAILED
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
                                &card);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Falha ao montar sistema de arquivos. "
                    "Se quiser que o cartão SD seja formatado, defina a"
                    "opção CONFIG_FORMAT_IF_MOUNT_FAILED no menuconfig.");
    } else {
      ESP_LOGE(
          TAG,
          "Falha ao inicializar cartão (%s). "
          "Verifique se os pinos do cartão SD estão com os resistores pull-up.",
          esp_err_to_name(ret));
    }
    return ret;
  }
  ESP_LOGI(TAG, "Montagem do sistema de arquivos concluída");
}

esp_err_t datalogger_deinit(const char *mount_point, sdmmc_card_t *card,
                            const sdmmc_host_t *host) {
  esp_err_t ret;
  // All done, unmount partition and disable SPI peripheral
  ret = esp_vfs_fat_sdcard_unmount(mount_point, card);
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_INVALID_ARG) {
      ESP_LOGE(TAG, "card argument is unregistered");
    } else {
      ESP_LOGE(TAG, "esp_vfs_fat_sdmmc_mount hasn't been called");
    }
    return ret;
  }
  ESP_LOGI(TAG, "Card unmounted");

  // deinitialize the bus after all devices are removed
  ret = spi_bus_free(host->slot);

  if (ret != ESP_OK) {
    if (ret == ESP_ERR_INVALID_ARG) {
      ESP_LOGE(TAG, "parameter is invalid");
    } else {
      ESP_LOGE(TAG, "bus hasn't been initialized before, or not all devices on "
                    "the bus are freed");
    }
    return ret;
  }
  return ESP_OK;
}

esp_err_t datalogger_append_to_file(const char *path, char *data) {
  ESP_LOGI(TAG, "Abrindo arquivo %s", path);
  FILE *f = fopen(path, "a");
  if (f == NULL) {
    ESP_LOGE(TAG, "Falha ao abrir arquivo para inserção");
    return ESP_FAIL;
  }
  fprintf(f, "%s", data);
  fclose(f);
  ESP_LOGI(TAG, "Inserção no arquivo concluída");

  return ESP_OK;
}

void app_main(void) {}
