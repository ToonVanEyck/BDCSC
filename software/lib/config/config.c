#include "config.h"
#include "debug_io.h"
#include "flash.h"

extern uint32_t __FLASH_NVS_START__;
#define NVS_START_ADDR ((uint32_t)&__FLASH_NVS_START__)

void configLoad(nvs_config_t *config)
{
    flashRead(NVS_START_ADDR, (uint8_t *)config, sizeof(nvs_config_t));
}

void configStore(nvs_config_t *config)
{
    flashWrite(NVS_START_ADDR, (uint8_t *)config, sizeof(nvs_config_t));
}

void configPrint(nvs_config_t *config)
{
}