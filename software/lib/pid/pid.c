#include "pid.h"

void pid_init(pid_ctx_t *pid, int32_t kp, int32_t ki, int32_t kd)
{
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->setpoint       = 0;
    pid->integral       = 0;
    pid->previous_error = 0;
    pid->o_min          = -10000;
    pid->o_max          = 10000;
}

int32_t pid_compute(pid_ctx_t *pid, int32_t error)
{
    // Calculate new integral with error
    if (error == 0) {
        pid->integral = pid->integral * 99 / 100; // Natural decay of integral term
    }
    pid->integral += error;

    int32_t derivative  = error - pid->previous_error;
    pid->previous_error = error;

    pid->p_error = (pid->kp * error);
    pid->i_error = (pid->ki * pid->integral);
    pid->d_error = (pid->kd * derivative);

    int32_t o_max = (-5 < error && error < 5) ? (pid->o_max * 5) : (pid->o_max * 1);
    int32_t o_min = (-5 < error && error < 5) ? (pid->o_min * 5) : (pid->o_min * 1);

    // Anti-windup: Check scaled integral value against limits
    if (pid->i_error > o_max) {
        // Back-calculate the raw integral value that would produce max
        pid->integral = o_max / pid->ki;
    } else if (pid->i_error < o_min) {
        // Back-calculate the raw integral value that would produce min
        pid->integral = o_min / pid->ki;
    }
    pid->i_error = (pid->ki * pid->integral);

    pid->output = (pid->p_error + pid->i_error + pid->d_error) / 1000;

    return pid->output;
}
