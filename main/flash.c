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
    void (*tx_rx)(struct spi_dev *dev, uint8_t *tx_data, uint8_t *rx_data, uint32_t sz);
    void (*rst)(struct spi_dev *dev, bool assert);
};

static void spi_tx_rx(struct spi_dev *dev, uint8_t *tx_data,
		uint8_t *rx_data, uint32_t sz) {
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length = sz << 3;

    if (sz <= 4) {
	if (tx_data) {
	    trans.flags |= SPI_TRANS_USE_TXDATA;
	    memcpy(trans.tx_data, tx_data, sz);
	}
	if (rx_data) {
	    trans.flags |= SPI_TRANS_USE_RXDATA;
	    trans.rxlength = sz << 3;
	}
    } else {
	trans.tx_buffer = tx_data;
	trans.rx_buffer = rx_data;
    }
    ESP_ERROR_CHECK(spi_device_transmit(dev->spidev_hndl, &trans));
    if (sz <= 4 && rx_data)
	memcpy(rx_data, &trans.rx_data, sz);
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
	.flags = 0,//SPI_DEVICE_HALFDUPLEX
	.duty_cycle_pos = 0,
	.input_delay_ns = 0,
	.pre_cb = NULL,
	.post_cb = NULL,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &spidev, &dev->spidev_hndl));

    dev->tx_rx = spi_tx_rx;
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

#define STM32_ACK	0x79
#define STM32_NACK	0x1f
#define STM32_SOF	0x5a
#define STM32_RSOF	0xa5
#define STM32_READ	0x11
#define STM32_WRITE	0x31
#define STM32_ERASE	0x44


static int stm32_sync(struct spi_dev *spi) {
    uint8_t buf = STM32_SOF;
    spi->tx_rx(spi, &buf, NULL, 1);
    spi->tx_rx(spi, NULL, &buf, 1);
    if (buf != STM32_RSOF) {
	printf("Failed to sync\n");
	return 1;
    }
    return 0;
}

static int stm32_get_ack(struct spi_dev *spi) {
    int tries = 1000;
    uint8_t buf = 0x0;
    spi->tx_rx(spi, &buf, NULL, 1);
    for (int i = 0; i < tries; i++) {
	spi->tx_rx(spi, NULL, &buf, 1);
	if (buf == STM32_ACK) {
	    spi->tx_rx(spi, &buf, NULL, 1);
printf("With %d try\n", i);
	    return 0;
	} else if (buf == STM32_NACK) {
	    printf("Got NACK\n");
	    return 1;
	}
    }
    return 1;
}

static int stm32_send_cmd(struct spi_dev *spi, uint8_t cmd) {
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];

    tx_buf[0] = STM32_SOF;
    tx_buf[1] = cmd;
    tx_buf[2] = ~cmd;
    spi->tx_rx(spi, tx_buf, rx_buf, 3);   

    if (stm32_get_ack(spi)) {
	printf("Failed to get cmd %02x ack\n", cmd);
	return 1;
    }
    return 0;
}

static int stm32_read_mem(struct spi_dev *spi, uint32_t addr, uint8_t *data, uint32_t sz) {
    uint8_t buf[5];
    uint8_t rx_buf[5];
    if (stm32_send_cmd(spi, STM32_READ)) {
	printf("Failed to read\n");
	return 1;
    }
    buf[4] = 0;
    for (int i = 0; i < 4; i++) {
	buf[i] = ((uint8_t *)&addr)[3 - i];
	buf[4] ^= buf[i];
    }

    spi->tx_rx(spi, buf, NULL, 5);
    if (stm32_get_ack(spi)) {
	printf("Failed to get read ack\n");
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

static int stm32_write_mem(struct spi_dev *spi, uint32_t addr, uint8_t *data, uint32_t sz) {
    uint8_t buf[5];
    if (stm32_send_cmd(spi, STM32_WRITE)) {
	printf("Failed to write\n");
	return 1;
    }
    buf[4] = 0;
    for (int i = 0; i < 4; i++) {
	buf[i] = ((uint8_t *)&addr)[3 - i];
	buf[4] ^= buf[i];
    }

    spi->tx_rx(spi, buf, NULL, 5);
    if (stm32_get_ack(spi)) {
	printf("Failed to get write ack\n");
	return 1;
    }

    buf[0] = (uint8_t)(sz -1);
    spi->tx_rx(spi, buf, NULL, 1);

    spi->tx_rx(spi, data, NULL, sz);

    buf[0] = sz - 1;
    for (int i = 0; i < sz; i++)
	buf[0] ^= data[i];
    spi->tx_rx(spi, buf, NULL, 1);

    if (stm32_get_ack(spi)) {
	printf("Failed to get ack\n");
	return 1;
    }
    return 0;
};

static int stm32_erase_mem(struct spi_dev * spi, uint32_t addr, uint32_t sz) {
    uint8_t buf[5];
    if (stm32_send_cmd(spi, STM32_ERASE)) {
	printf("Failed to erase\n");
	return 1;
    }
    buf[0] = 0xff; 
    buf[1] = 0xff;
    buf[2] = buf[0] ^ buf[1];
    spi->tx_rx(spi, buf, NULL, 3);

    if (stm32_get_ack(spi)) {
	printf("Failed to get erase ack\n");
	return 1;
    }
    return 0;
}

void stm32_flash_task(void *p) {
    struct spi_dev spi;
    uint8_t buf[32] = { 0x0 };
    memset(buf, 0x0, sizeof(buf));
    uint8_t write_buf[0x100];
    memset(write_buf, 0x69, sizeof(write_buf));
    uint8_t read_buf[0x100];
    memset(read_buf, 0, sizeof(read_buf));

    spi_init(&spi);
    spi.rst(&spi, true);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    if (stm32_sync(&spi))
	return;
    if (stm32_get_ack(&spi))
	return;

    if (stm32_send_cmd(&spi, 0x2))
	return;

    spi.tx_rx(&spi, NULL, buf, sizeof(buf));

    for (int i = 0; i < sizeof(buf); i++) {
	printf("%02x ", buf[i]);
    }
    printf("\n");

    if (stm32_erase_mem(&spi, 0x8000000, 0x100))
	printf("Erase Failed\n");
    write_buf[0] = write_buf[0xff] = 0x55;
    if (stm32_write_mem(&spi, 0x8000000, write_buf, 0x100))
	printf("Write Failed\n");
    if (stm32_read_mem(&spi, 0x8000000, read_buf, 0x100))
	printf("Reade Failed\n");

    printf("WRITE:\n");
    for (int i = 0; i < 0x100; i++)
	printf("%02x ", write_buf[i]);
    printf("\n");
    printf("READ:\n");
    for (int i = 0; i < 0x100; i++)
	printf("%02x ", read_buf[i]);
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
