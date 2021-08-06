#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi.h"
#include "stm32_flasher.h"

void app_main(void) {
    struct spi_dev spidev;
    struct flasher_dev flashdev;
    spi_init(&spidev);
    spawn_flash_task(&flashdev, &spidev, "image.bin");
    while(1)
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}
