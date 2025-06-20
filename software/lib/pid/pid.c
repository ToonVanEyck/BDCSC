#include "pid.h"

void pid_init(pid_ctx_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = 0;
    pid->integral       = 0;
    pid->previous_error = 0;
    pid->integral_min   = INT32_MIN;
    pid->integral_max   = INT32_MAX;
}

int32_t pid_compute(pid_ctx_t *pid, int32_t measured_value)
{
    int32_t error = pid->setpoint - measured_value;
    pid->integral += error;
    // Anti-windup: Clamp the integral term
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    int32_t derivative  = error - pid->previous_error;
    pid->previous_error = error;

    return ((pid->kp * error) + (pid->ki * pid->integral) / 100 + (pid->kd * derivative) / 100) / 1000;
}
