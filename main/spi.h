#ifndef __SPI_H__
#define __SPI_H__
#include "driver/spi_master.h"

struct spi_dev {
    spi_device_handle_t spidev_hndl;
    void (*tx_rx)(struct spi_dev *dev, uint8_t *tx_data, uint8_t *rx_data, uint32_t sz);
};
void spi_init(struct spi_dev *dev);

#endif
