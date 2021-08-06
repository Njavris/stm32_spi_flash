#include <string.h>
#include "sdkconfig.h"
#include "spi.h"

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
	.flags = 0,
	.duty_cycle_pos = 0,
	.input_delay_ns = 0,
	.pre_cb = NULL,
	.post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &spidev, &dev->spidev_hndl));

    dev->tx_rx = spi_tx_rx;
}
