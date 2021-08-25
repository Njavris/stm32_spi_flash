#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <driver/gpio.h>
#include "esp_log.h"
#include "stm32_flasher.h"
#include "esp_spiffs.h"

static const char *STM32_FLASHER_TAG = "STM32_FLASHER";

#define STM32_ACK	0x79
#define STM32_NACK	0x1f
#define STM32_SOF	0x5a
#define STM32_RSOF	0xa5
#define STM32_READ	0x11
#define STM32_WRITE	0x31
#define STM32_ERASE	0x44

#define STM32_ERASE_ALL		0xffff
#define STM32_ERASE_BANK1	0xfffe
#define STM32_ERASE_BANK2	0xfffd

static enum flasher_result stm32_flasher_status = FLASHER_STATUS_MAX;
SemaphoreHandle_t flasher_status_mutex = NULL;

static void stm32_reset(struct flasher_dev *dev, bool assert) {
    gpio_set_level(dev->bootm_pin, !(dev->bootm_pol ^ !!assert));

    gpio_set_level(dev->rst_pin, dev->rst_pol ^ false);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(dev->rst_pin, dev->rst_pol ^ true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

int flasher_init(struct flasher_dev *dev, struct spi_dev *spi, char *filename, uint32_t address) {
    xSemaphoreTake(flasher_status_mutex, portMAX_DELAY);
    if (stm32_flasher_status && stm32_flasher_status != FLASHER_STATUS_MAX)
	return 1;
    xSemaphoreGive(flasher_status_mutex);
    memset(dev, 0, sizeof(struct flasher_dev));
    dev->spidev = spi;
    dev->rst_pin = CONFIG_STM32_PIN_RESET;
    dev->rst_pol = CONFIG_STM32_POL_RESET;
    dev->bootm_pin = CONFIG_STM32_PIN_BOOTM;
    dev->bootm_pol = CONFIG_STM32_POL_BOOTM;
    dev->fn = filename;
    dev->address = address;

    gpio_set_direction(dev->bootm_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->rst_pin, GPIO_MODE_OUTPUT);
    return 0;
}

static int stm32_sync(struct flasher_dev *dev) {
    struct spi_dev *spi = dev->spidev;
    uint8_t buf = STM32_SOF;
    spi->tx_rx(spi, &buf, NULL, 1);
    spi->tx_rx(spi, NULL, &buf, 1);
    if (buf != STM32_RSOF) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to sync\n");
	return 1;
    }
    return 0;
}

static int stm32_get_ack(struct flasher_dev *dev) {
    struct spi_dev *spi = dev->spidev;
    int tries = 100;
    uint8_t buf = 0x0;
    spi->tx_rx(spi, &buf, NULL, 1);
    for (int i = 0; i < tries; i++) {
	spi->tx_rx(spi, NULL, &buf, 1);
	if (buf == STM32_ACK) {
	    spi->tx_rx(spi, &buf, NULL, 1);
	    return 0;
	} else if (buf == STM32_NACK) {
	    ESP_LOGE(STM32_FLASHER_TAG, "Got NACK\n");
	    return 1;
	}
	vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return 1;
}

static int stm32_send_cmd(struct flasher_dev *dev, uint8_t cmd) {
    struct spi_dev *spi = dev->spidev;
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];

    tx_buf[0] = STM32_SOF;
    tx_buf[1] = cmd;
    tx_buf[2] = ~cmd;
    spi->tx_rx(spi, tx_buf, rx_buf, 3);   

    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get cmd %02x ack\n", cmd);
	return 1;
    }
    return 0;
}

static int stm32_read_mem(struct flasher_dev *dev, uint32_t addr, uint8_t *data, uint32_t sz) {
    struct spi_dev *spi = dev->spidev;
    uint8_t buf[5];
    uint8_t rx_buf[5];
    if (stm32_send_cmd(dev, STM32_READ)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to read\n");
	return 1;
    }
    buf[4] = 0;
    for (int i = 0; i < 4; i++) {
	buf[i] = ((uint8_t *)&addr)[3 - i];
	buf[4] ^= buf[i];
    }

    spi->tx_rx(spi, buf, NULL, 5);
    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get read ack\n");
	return 1;
    }

    buf[0] = (uint8_t)(sz -1);
    buf[1] = ~buf[0];
    spi->tx_rx(spi, buf, rx_buf, 2);
    while (rx_buf[0] != STM32_ACK)
	spi->tx_rx(spi, NULL, rx_buf, 1);
    spi->tx_rx(spi, NULL, rx_buf, 2);

    spi->tx_rx(spi, NULL, data, sz);
    spi->tx_rx(spi, NULL, buf, 1);

    return 0;
};

static int stm32_write_mem(struct flasher_dev *dev, uint32_t addr, uint8_t *data, uint32_t sz) {
    struct spi_dev *spi = dev->spidev;
    uint8_t buf[5];
    if (stm32_send_cmd(dev, STM32_WRITE)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to write\n");
	return 1;
    }
    buf[4] = 0;
    for (int i = 0; i < 4; i++) {
	buf[i] = ((uint8_t *)&addr)[3 - i];
	buf[4] ^= buf[i];
    }

    spi->tx_rx(spi, buf, NULL, 5);
    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get write ack\n");
	return 1;
    }

    buf[0] = (uint8_t)(sz -1);
    spi->tx_rx(spi, buf, NULL, 1);

    spi->tx_rx(spi, data, NULL, sz);

    buf[0] = sz - 1;
    for (int i = 0; i < sz; i++)
	buf[0] ^= data[i];
    spi->tx_rx(spi, buf, NULL, 1);

    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get ack\n");
	return 1;
    }
    return 0;
};

static int stm32_erase_mem(struct flasher_dev *dev, uint16_t page) {
    struct spi_dev *spi = dev->spidev;
    uint8_t buf[5];
    if (stm32_send_cmd(dev, STM32_ERASE)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to erase\n");
	return 1;
    }
    if (page == STM32_ERASE_ALL ||
		page == STM32_ERASE_BANK1 ||
		page == STM32_ERASE_BANK2) {
	buf[0] = (page >> 8) & 0xff; 
	buf[1] = page & 0xff;
    } else {
	buf[0] = 0;
	buf[1] = 0;
    }
    buf[2] = buf[0] ^ buf[1];
    spi->tx_rx(spi, buf, NULL, 3);

    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get erase num ack\n");
	return 1;
    }

    if (page == STM32_ERASE_ALL ||
		page == STM32_ERASE_BANK1 ||
		page == STM32_ERASE_BANK2)
	return 0;

    buf[0] = (page >> 8) & 0xff;
    buf[1] = page & 0xff;
    buf[2] = buf[0] ^ buf[1] ^ buf[2];
    spi->tx_rx(spi, buf, NULL, 3);

    if (stm32_get_ack(dev)) {
	ESP_LOGE(STM32_FLASHER_TAG, "Failed to get erase ack\n");
	return 1;
    }
    return 0;
}

#define PAGE_SZ				0x1000
#define ADDR_TO_PAGE(addr, base)	(((addr) - (base)) / PAGE_SZ)

int stm32_write_file(struct flasher_dev *dev) {
    uint32_t addr = dev->address;
    uint8_t buf[0x100];
    int nbytes;
    FILE *f = fopen(dev->fn, "rb");
    if (!f) {
        ESP_LOGE(STM32_FLASHER_TAG, "Failed to open file for reading");
        return 1;
    }
    ESP_LOGI(STM32_FLASHER_TAG, "Writing file %s\n", dev->fn);
    printf("Writing\n");
    while ((nbytes = fread(buf, 1, sizeof(buf), f)) > 0) {
	printf("\033[1A\033[K");
	printf("\b\rWriting: %x\n", addr);
	if (!(addr % 0x1000))
	    stm32_erase_mem(dev, ADDR_TO_PAGE(addr, dev->address));
	stm32_write_mem(dev, addr, buf, nbytes);
	addr += nbytes;
    }
    fclose(f);
    return 0;
}

int stm32_write_file_verify(struct flasher_dev *dev) {
    uint32_t addr = dev->address;
    uint8_t buf[0x100], rbuf[0x100];
    int nbytes;
    FILE *f = fopen(dev->fn, "rb");
    if (!f) {
        ESP_LOGE(STM32_FLASHER_TAG, "Failed to open file for reading");
        return 1;
    }
    ESP_LOGI(STM32_FLASHER_TAG, "Verifying file %s\n", dev->fn);
    printf("Verifying\n");
    while ((nbytes = fread(buf, 1, sizeof(buf), f)) > 0) {
	printf("\033[1A\033[K");
	printf("\b\rVerifying: %x\n", addr);
	stm32_read_mem(dev, addr, rbuf, nbytes);
	if (memcmp(rbuf, buf, sizeof(rbuf)))
	    return 1;
	addr += nbytes;
    }
    fclose(f);
    return 0;
}

void stm32_flash_task(void *p) {
    struct flasher_dev *dev = (struct flasher_dev *)p;
    xSemaphoreTake(flasher_status_mutex, portMAX_DELAY);
    stm32_flasher_status = FLASHER_RUNNING; 
    xSemaphoreGive(flasher_status_mutex);
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
	goto done;

    stm32_reset(dev, true);
    if (stm32_sync(dev))
	goto done;
    if (stm32_get_ack(dev))
	goto done;

    ESP_LOGI(STM32_FLASHER_TAG, "Erasing mem\n");
    stm32_erase_mem(dev, STM32_ERASE_ALL);

    stm32_write_file(dev);
    if (stm32_write_file_verify(dev)) {
	ESP_LOGI(STM32_FLASHER_TAG, "Verification failed\n");
	goto done;
    } else {
	ESP_LOGI(STM32_FLASHER_TAG, "Verified\n");
    }
    
    stm32_reset(dev, false);
    xSemaphoreTake(flasher_status_mutex, portMAX_DELAY);
    stm32_flasher_status = FLASHER_SUCCESS;
    xSemaphoreGive(flasher_status_mutex);
done:
    ESP_LOGI(STM32_FLASHER_TAG, "%s done\n", __func__);
    esp_vfs_spiffs_unregister(conf.partition_label);
    xSemaphoreTake(flasher_status_mutex, portMAX_DELAY);
    if (stm32_flasher_status)
	stm32_flasher_status = FLASHER_FAILED;
    xSemaphoreGive(flasher_status_mutex);
    vTaskDelete(NULL);
}

void spawn_flash_task(struct flasher_dev *dev, struct spi_dev *spi, char *filename, uint32_t address) {
    flasher_status_mutex = xSemaphoreCreateMutex();
    if (flasher_init(dev, spi, filename, address))
	return;
    xTaskCreate(stm32_flash_task, "STM32_flash_task", CONFIG_STM32_TASK_FRAME,
			dev, 1, &dev->flash_task_hndl);
}


enum flasher_result get_flasher_result(struct flasher_dev *dev) {
    enum flasher_result ret;
    xSemaphoreTake(flasher_status_mutex, portMAX_DELAY);
    ret = stm32_flasher_status;
    xSemaphoreGive(flasher_status_mutex);
    return ret;
}
