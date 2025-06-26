#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "debug_io.h"

#define MOTOR_GEAR_RATIO     (298)   /**< Gear ratio of the motor. */
#define MOTOR_ENCODER_PPR    (7 * 4) /**< Encoder pulses per revolution before gear reduction. */
#define MOTOR_SENSORLESS_PPR (6)     /**< Sensorless encoder pulses per revolution before gear reduction. */
