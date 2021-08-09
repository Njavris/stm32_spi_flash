#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include "esp_log.h"
#include "stm32_flasher.h"

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

static void stm32_reset(struct flasher_dev *dev, bool assert) {
    gpio_set_level(dev->bootm_pin, !(dev->bootm_pol ^ !!assert));

    gpio_set_level(dev->rst_pin, dev->rst_pol ^ false);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(dev->rst_pin, dev->rst_pol ^ true);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void flasher_init(struct flasher_dev *dev, struct spi_dev *spi, char *filename) {
    memset(dev, 0, sizeof(struct flasher_dev));
    dev->spidev = spi;
    dev->rst_pin = CONFIG_STM32_PIN_RESET;
    dev->rst_pol = CONFIG_STM32_POL_RESET;
    dev->bootm_pin = CONFIG_STM32_PIN_BOOTM;
    dev->bootm_pol = CONFIG_STM32_POL_BOOTM;
    dev->fn = filename;

    gpio_set_direction(dev->bootm_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->rst_pin, GPIO_MODE_OUTPUT);
//    gpio_set_pull_mode(dev->bootm_pin, GPIO_PULLUP_ONLY);
//    gpio_set_pull_mode(dev->rst_pin, GPIO_PULLUP_ONLY);
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
ESP_LOGI(STM32_FLASHER_TAG, "ACK in %d tries\n", i);
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

void test_write(struct flasher_dev *dev, uint8_t val) {
    uint8_t write_buf[0x100];
    uint8_t read_buf[0x100];
    memset(read_buf, 0, sizeof(read_buf));
    memset(write_buf, val, sizeof(write_buf));
    if (stm32_write_mem(dev, 0x8000000, write_buf, 0x100))
	ESP_LOGE(STM32_FLASHER_TAG, "Write Failed\n");
    if (stm32_read_mem(dev, 0x8000000, read_buf, 0x100))
	ESP_LOGE(STM32_FLASHER_TAG, "Read Failed\n");
    printf("WRITE:\n");
    for (int i = 0; i < 0x100; i++)
	printf("%02x ", write_buf[i]);
    printf("\n");
    printf("READ:\n");
    for (int i = 0; i < 0x100; i++)
	printf("%02x ", read_buf[i]);
    printf("\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void stm32_flash_task(void *p) {
    struct flasher_dev *dev = (struct flasher_dev *)p;


    stm32_reset(dev, true);
    if (stm32_sync(dev))
	goto fail;
    if (stm32_get_ack(dev))
	goto fail;

    if (stm32_erase_mem(dev, STM32_ERASE_ALL))
	ESP_LOGE(STM32_FLASHER_TAG, "Erase Failed\n");
    test_write(dev, 1);
    if (stm32_erase_mem(dev, 1))
	ESP_LOGE(STM32_FLASHER_TAG, "Erase Failed\n");
    test_write(dev, 2);
    if (stm32_erase_mem(dev, 2))
	ESP_LOGE(STM32_FLASHER_TAG, "Erase Failed\n");
    test_write(dev, 3);
    if (stm32_erase_mem(dev, STM32_ERASE_BANK1))
	ESP_LOGE(STM32_FLASHER_TAG, "Erase Failed\n");
    test_write(dev, 4);
    if (stm32_erase_mem(dev, STM32_ERASE_BANK2))
	ESP_LOGE(STM32_FLASHER_TAG, "Erase Failed\n");
    test_write(dev, 5);

    
    stm32_reset(dev, false);

fail:
    ESP_LOGI(STM32_FLASHER_TAG, "%s done\n", __func__);
    vTaskDelete(NULL);
}

void spawn_flash_task(struct flasher_dev *dev, struct spi_dev *spi, char *filename) {
    TaskHandle_t flash_task_hndl = NULL;
    flasher_init(dev, spi, filename);
    xTaskCreate(stm32_flash_task, "STM32_flash_task", CONFIG_STM32_TASK_FRAME,
			dev, 1, &flash_task_hndl);
}
