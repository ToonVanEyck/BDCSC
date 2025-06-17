#pragma once
#include <stdbool.h>
#include <stdint.h>

/** Configuration data for NVM storage. */
typedef struct nvs_config_tag {
    bool ota_completed; /**< Flag to indicate that the OTA process is completed. */
} nvs_config_t;

/** Load the config form NVM. */
void configLoad(nvs_config_t *config);

/** Store the config in NVM. */
void configStore(nvs_config_t *config);

/** Print the config to RTT. */
void configPrint(nvs_config_t *config);