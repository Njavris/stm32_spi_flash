#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi.h"
#include "stm32_flasher.h"
#include "esp_spiffs.h"

void app_main(void) {
    struct spi_dev spidev;
    struct flasher_dev flashdev;

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    spi_init(&spidev);
    spawn_flash_task(&flashdev, &spidev, "/spiffs/image.bin");

    esp_vfs_spiffs_unregister(conf.partition_label);
    while(1) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
