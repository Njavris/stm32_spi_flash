idf_component_register(SRCS
	"main.c"
	"spi.c"
	"stm32_flasher.c"
	INCLUDE_DIRS "")


spiffs_create_partition_image(storage ../spiffs_dir FLASH_IN_PROJECT)
