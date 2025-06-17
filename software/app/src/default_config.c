#include "config.h"

#ifdef SET_DEFAULT_CONFIG
/* When SET_DEFAULT_CONFIG is not defined, the default configuration will not be included in the final binary. */
const nvs_config_t __attribute__((section(".config"))) config = {
    .ota_completed = false,
};
#endif