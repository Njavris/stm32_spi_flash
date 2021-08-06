#ifndef __FLASH_H__
#define __FLASH_H__
#include "spi.h"

struct flasher_dev {
    struct spi_dev *spidev;
    int rst_pin;
    int rst_pol;
    int bootm_pin;
    int bootm_pol;
    char *fn;
};

void spawn_flash_task(struct flasher_dev *dev, struct spi_dev *spi, char *filename);

#endif
