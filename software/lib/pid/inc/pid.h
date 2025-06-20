#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t setpoint;
    int32_t integral;
    int32_t previous_error;
    int32_t integral_min; // Minimum value for integral term (anti-windup)
    int32_t integral_max; // Maximum value for integral term (anti-windup)
} pid_ctx_t;

void pid_init(pid_ctx_t *pid, int32_t kp, int32_t ki, int32_t kd);

int32_t pid_compute(pid_ctx_t *pid, int32_t measured_value);