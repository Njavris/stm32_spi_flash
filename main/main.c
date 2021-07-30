/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
    gpio_set_level(dev->rst_pin, !(dev->rst_pol ^ !!assert));
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
	.queue_size = 32,
	.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_BIT_LSBFIRST,
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
    dev->rst_pin = CONFIG_STM32_PIN_BOOT;
    dev->rst_pol = CONFIG_STM32_POL_BOOT;

    gpio_set_direction(dev->rst_pin, GPIO_MODE_OUTPUT);
}

void stm32_flash_task(void *p) {
    struct spi_dev spi;
    esp_err_t ret;

    spi_init(&spi);

    spi.rst(&spi, true);

    uint8_t buf[4];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x5a;
    spi.tx(&spi, buf, 2);
    spi.rx(&spi, buf, 2);
    printf("buf =");
    for (int i = 0; i < sizeof(buf); i++)
	printf(" %02x", buf[i]);
    printf("\n");
    
    spi.rst(&spi, false);

    while(1) {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    TaskHandle_t flash_task_hndl = NULL;
    xTaskCreate(stm32_flash_task, "STM32_flash_task", CONFIG_STM32_TASK_FRAME,
			NULL, 1, &flash_task_hndl);
}
