#ifndef __FLASH_H__
#define __FLASH_H__
#include "spi.h"

struct flasher_dev {
    TaskHandle_t flash_task_hndl;
    struct spi_dev *spidev;
    int status;
    int rst_pin;
    int rst_pol;
    int bootm_pin;
    int bootm_pol;
    char *fn;
    uint32_t address;
};

enum flasher_result {
    FLASHER_SUCCESS = 0,
    FLASHER_FAILED,
    FLASHER_RUNNING,
    FLASHER_STATUS_MAX,
};

void spawn_flash_task(struct flasher_dev *dev, struct spi_dev *spi,
					char *filename, uint32_t address);
enum flasher_result get_flasher_result(struct flasher_dev *dev);

#endif
