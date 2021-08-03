#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include <driver/gpio.h>

#define STM32_FLASH_TAG	"STM32_FLASH"

struct spi_dev {
    spi_device_handle_t spidev_hndl;
    int rst_pin;
    int rst_pol;
    int bootm_pin;
    int bootm_pol;
    void (*tx)(struct spi_dev *dev, uint8_t *data, uint32_t sz);
    void (*rx)(struct spi_dev *dev, uint8_t *data, uint32_t sz);
    void (*rst)(struct spi_dev *dev, bool assert);
};

static void spi_tx(struct spi_dev *dev, uint8_t *data, uint32_t sz) {
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.length = sz << 3;
    memcpy(trans.tx_data, data, sz);
    ESP_ERROR_CHECK(spi_device_transmit(dev->spidev_hndl, &trans));
}

static void spi_rx(struct spi_dev *dev, uint8_t *data, uint32_t sz) {
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.rxlength = sz << 3;
    ESP_ERROR_CHECK(spi_device_transmit(dev->spidev_hndl, &trans));
    memcpy(data, &trans.rx_data, sz);
}

static void stm32_reset(struct spi_dev *dev, bool assert) {
    gpio_set_level(dev->bootm_pin, !(dev->bootm_pol ^ !!assert));

    gpio_set_level(dev->rst_pin, dev->rst_pol ^ false);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(dev->rst_pin, dev->rst_pol ^ true);
}

void spi_init(struct spi_dev *dev) {
    memset(dev, 0, sizeof(struct spi_dev));

    spi_bus_config_t buscfg = {
	.miso_io_num = CONFIG_SPI_PIN_MISO,
	.mosi_io_num = CONFIG_SPI_PIN_MOSI,
	.sclk_io_num = CONFIG_SPI_PIN_CLK,
	.quadwp_io_num = -1,
	.quadhd_io_num = -1,
	.max_transfer_sz = 0,
    };
    spi_device_interface_config_t spidev = {
	.clock_speed_hz = CONFIG_STM32_SPI_SPEED,
	.spics_io_num = CONFIG_SPI_PIN_STM32_CS,
	.mode = 0,
	.cs_ena_pretrans = 0,
	.cs_ena_posttrans = 0,
	.command_bits = 0,
	.address_bits = 0,
	.dummy_bits = 0,
	.queue_size = 4,
	.flags = SPI_DEVICE_HALFDUPLEX,
	.duty_cycle_pos = 0,
	.input_delay_ns = 0,
	.pre_cb = NULL,
	.post_cb = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &spidev, &dev->spidev_hndl));

    dev->tx = spi_tx;
    dev->rx = spi_rx;
    dev->rst = stm32_reset;
    dev->rst_pin = CONFIG_STM32_PIN_RESET;
    dev->rst_pol = CONFIG_STM32_POL_RESET;
    dev->bootm_pin = CONFIG_STM32_PIN_BOOTM;
    dev->bootm_pol = CONFIG_STM32_POL_BOOTM;

    gpio_set_direction(dev->bootm_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->rst_pin, GPIO_MODE_OUTPUT);
//    gpio_set_pull_mode(dev->bootm_pin, GPIO_PULLUP_ONLY);
//    gpio_set_pull_mode(dev->rst_pin, GPIO_PULLUP_ONLY);
}

static int stm32_sync(struct spi_dev *spi) {
    uint8_t buf = 0x5a;
    spi->tx(spi, &buf, 1);
    spi->rx(spi, &buf, 1);
    if (buf != 0xa5) {
	printf("Failed to sync\n");
	return 1;
    }
    return 0;
}

static int stm32_get_ack(struct spi_dev *spi, int tries) {
    uint8_t buf = 0x0;
    spi->tx(spi, &buf, 1);
    for (int i = 0; i < tries; i++) {
	spi->rx(spi, &buf, 1);
	if (buf == 0x79) {
	    spi->tx(spi, &buf, 1);
	    return 0;
	}
    }
    printf("Failed to get ack in %d tries\n", tries);
    return 1;
}

static int stm32_send_cmd(struct spi_dev *spi, uint8_t cmd) {
    uint8_t buf = 0x5a;
    spi->tx(spi, &buf, 1);
    spi->tx(spi, &cmd, 1);
    spi->rx(spi, &buf, 1);
    if (buf != 0x79) {
	printf("Failed to send cmd %02x\n", cmd);
	return 1;
    }

    if (stm32_get_ack(spi, 100)) {
	printf("Failed to get cmd %02x ack\n", cmd);
	return 1;
    }
    return 0;
}

void stm32_flash_task(void *p) {
    struct spi_dev spi;
//    esp_err_t ret;

    spi_init(&spi);
    spi.rst(&spi, true);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (stm32_sync(&spi))
	return;
    if (stm32_get_ack(&spi, 100))
	return;

    if (stm32_send_cmd(&spi, 0x1))
	return;

    uint32_t buf[32];
    spi.rx(&spi, buf, sizeof(buf));
    for (int i = 0; i < sizeof(buf); i++) {
	printf("%02x ", buf[i]);
    }
    printf("\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    spi.rst(&spi, false);

    while(1) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void spawn_flash_task(void) {
    TaskHandle_t flash_task_hndl = NULL;
    xTaskCreate(stm32_flash_task, "STM32_flash_task", CONFIG_STM32_TASK_FRAME,
			NULL, 1, &flash_task_hndl);
}
