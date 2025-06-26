/* Includes ------------------------------------------------------------------*/
#include "bdcsc.h"
#include "peripherals.h"
#include "rbuff.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/

#ifndef VERSION
#define VERSION "not found"
#endif

/* Private variables ---------------------------------------------------------*/
// ADC_HandleTypeDef AdcHandle;
// ADC_ChannelConfTypeDef sConfig;
// uint32_t aADCxConvertedData[ENCODER_CHANNEL_CNT] = {0};

// TIM_HandleTypeDef encoderHandle; // Encoder timer
// TIM_HandleTypeDef Tim1Handle;    // PWM
// COMP_HandleTypeDef Comp2Handle;  // Comparator 2

extern rbuff_t tim1_ic_dma_rb; /**< DMA buffer for TIM1 input capture. */

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void APP_ErrorHandler(void);

uint16_t encoder_position_get(void);

/* Debug terminal functions. */
void debug_term_motor_toggle(const char *input, void *arg);
void debug_term_motor_set_speed(const char *input, void *arg);
void debug_term_motor_position_set(const char *input, void *arg);
void debug_term_motor_set_direction(const char *input, void *arg);
void debug_term_encoder_toggle(const char *input, void *arg);
void debug_term_pid_set(const char *input, void *arg);
void debug_term_pid_print(const char *input, void *arg);
void debug_term_pid_toggle(const char *input, void *arg);
void debug_term_debug_pin(const char *input, void *arg);

/* Motor stuff */
static motor_t debug_motor           = {0};
static uint16_t debug_motor_position = 0; /**< Current motor position. */

/* Quad encoder */
static uint16_t prev_encoder_value = 0;
static bool enable_encoder         = false;

/* PID */
pid_ctx_t pid_ctx       = {0};
static bool pid_enabled = false;

/* Debug IO scope struct */
typedef struct {
    uint32_t setpoint;
    uint32_t actual;
    int32_t p;
    int32_t i;
    int32_t d;
    int32_t error;
} debug_io_scope_t;
static debug_io_scope_t debug_io_scope = {0};

int main(void)
{
    /* Initialize Hardware. */
    peripherals_init();

    /* Register terminal functions. */
    debug_io_term_register_keyword("m", debug_term_motor_toggle, &debug_motor);
    debug_io_term_register_keyword("ms", debug_term_motor_set_speed, &debug_motor);
    debug_io_term_register_keyword("md", debug_term_motor_set_direction, &debug_motor);
    debug_io_term_register_keyword("mp", debug_term_motor_position_set, &debug_motor);
    debug_io_term_register_keyword("enc", debug_term_encoder_toggle, NULL);
    debug_io_term_register_keyword("pid", debug_term_pid_set, &pid_ctx);
    debug_io_term_register_keyword("pe", debug_term_pid_print, &pid_ctx); /* Register the new function */
    debug_io_term_register_keyword("p", debug_term_pid_toggle, &pid_ctx);
    debug_io_term_register_keyword("d", debug_term_debug_pin, NULL);

    /* Initialize debug IO. */
    debug_io_init(LOG_LVL_DEBUG);
    debug_io_scope_init("u4u4i4i4i4i4");

    /* Startup messages. */
    debug_io_log_info("BDCSC dev board\n");
    debug_io_log_info("Version: %s\n", __GIT_VERSION__);
    debug_io_log_info("Compilation Date: %s %s\n", __DATE__, __TIME__);

    /* Initialize PID controller. */
    pid_init(&pid_ctx, 500, 40, -120);

    bool compensate_overshoot = true;
    uint32_t last_tick        = 0;

    debug_motor.enable = true; // Enable motor by default
    debug_motor.speed  = 100;  // Start with zero speed
    motor_control(&debug_motor);

    while (1) {

        /* Process debug IO terminal input. */
        debug_io_term_process();

        uint16_t encoder_count = encoder_position_get();
        if (encoder_count != prev_encoder_value) {
            prev_encoder_value = encoder_count;
            if (enable_encoder) {
                debug_io_log_info("ENC: %u\n", encoder_count);
            }
        }

        int32_t error = 0, shortest_error = 0, positive_error = 0;
        uint32_t current_tick = tim1_tick_count_get();
        if (current_tick - last_tick >= 10) { // 10 ticks = 50ms
            last_tick = current_tick;

            uint32_t dma_val = 0;
            while (rbuff_read(&tim1_ic_dma_rb, &dma_val, 1) > 0) {
                debug_io_log_info("DMA: %u\n", dma_val);
            }

            if (pid_enabled && debug_motor.enable) {

                /* Check for new setpoint. */
                if (pid_ctx.setpoint != debug_motor_position) {
                    compensate_overshoot = false;
                    pid_ctx.setpoint     = debug_motor_position;
                }

                const uint16_t MOTOR_PULSE_CNT = MOTOR_GEAR_RATIO * MOTOR_SENSORLESS_PPR;

                /* Calculate shortest error (can be negative or positive, wraps around) */
                shortest_error = pid_ctx.setpoint - encoder_count;
                if (shortest_error > MOTOR_PULSE_CNT / 2) {
                    shortest_error -= MOTOR_PULSE_CNT;
                } else if (shortest_error < -(MOTOR_PULSE_CNT / 2)) {
                    shortest_error += MOTOR_PULSE_CNT;
                }

                /* Calculate positive-only error (always positive, wraps around) */
                positive_error = (pid_ctx.setpoint + MOTOR_PULSE_CNT - encoder_count) % MOTOR_PULSE_CNT;

                /* Initially use the positive error, but allow negative errors to compensate overshoot. */
                if (!compensate_overshoot) {
                    error = positive_error; // Use shortest error directly
                    if (positive_error == shortest_error) {
                        compensate_overshoot = true; // Use positive error for overshoot compensation
                    }
                } else {
                    error = shortest_error; // Use shortest error for PID calculation
                }

                /* Clip error. */
                if (-3 <= error && error <= 3) {
                    error = 0;
                }

                /* Do PID calculation. */
                int16_t pid_output = pid_compute(&pid_ctx, error);

                /* Clip PID output. */
                if (pid_output > 0) {
                    debug_motor.direction_cw = true;
                    debug_motor.speed        = (uint8_t)(pid_output > 180 ? 180 : pid_output);
                } else {
                    debug_motor.direction_cw = false;
                    debug_motor.speed        = (uint8_t)(-pid_output > 180 ? 180 : -pid_output);
                }

                /* Clip motor speed. */
                if (debug_motor.speed < 10) {
                    debug_motor.speed = 0; // If speed is too low, stop the motor
                }

                /* Update motor speed. */
                motor_control(&debug_motor);
            }
            // debug_io_log_debug("Hello World!\n");
            debug_io_scope.setpoint = pid_ctx.setpoint;
            debug_io_scope.actual   = encoder_count;
            debug_io_scope.p        = pid_ctx.p_error;
            debug_io_scope.i        = pid_ctx.i_error;
            debug_io_scope.d        = pid_ctx.d_error;
            debug_io_scope.error    = debug_motor.speed; // pid_ctx.output;

            debug_io_scope_push(&debug_io_scope, sizeof(debug_io_scope));
        }
    }
}

void APP_ErrorHandler(void)
{
    while (1)
        ;
}

void debug_term_motor_toggle(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    motor->enable  = !motor->enable;
    debug_io_log_info("Motor: %s\n", motor->enable ? "enabled" : "disabled");
    motor_control(motor);
}

void debug_term_motor_set_speed(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    int speed      = atoi(input);
    if (speed < 0 || speed > 1000) {
        debug_io_log_info("Invalid speed value. Must be between 0 and 1000.\n");
        return;
    }
    motor->speed = (uint16_t)speed;
    debug_io_log_info("Motor speed set to: %u\n", motor->speed);
    motor_control(motor);
}

void debug_term_motor_set_direction(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    if (strcmp(input, "cw") == 0) {
        motor->direction_cw = true;
        debug_io_log_info("Motor direction set to: CW\n");
    } else if (strcmp(input, "ccw") == 0) {
        motor->direction_cw = false;
        debug_io_log_info("Motor direction set to: CCW\n");
    } else {
        debug_io_log_info("Invalid direction. Use 'cw' or 'ccw'.\n");
    }
    motor_control(motor);
}

void debug_term_motor_position_set(const char *input, void *arg)
{
    motor_t *motor = (motor_t *)arg;
    int position   = atoi(input);
    if (position < 0 || position > 1787) {
        debug_io_log_info("Invalid position value. Must be between 0 and 1787.\n");
        return;
    }
    debug_motor_position = (uint16_t)position;
    debug_io_log_info("Motor position set to: %u\n", debug_motor_position);
}

void debug_term_encoder_toggle(const char *input, void *arg)
{
    enable_encoder = !enable_encoder;
    debug_io_log_info("Encoder: %s\n", enable_encoder ? "enabled" : "disabled");
}

void debug_term_pid_set(const char *input, void *arg)
{
    pid_ctx_t *pid = (pid_ctx_t *)arg;
    int kp, ki, kd;
    if (sscanf(input, "%d %d %d", &kp, &ki, &kd) == 3) {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
        debug_io_log_info("PID gains set: kp=%d, ki=%d, kd=%d\n", kp, ki, kd);
    } else {
        debug_io_log_info("Usage: pid <kp> <ki> <kd>\n");
    }
}

void debug_term_pid_print(const char *input, void *arg)
{
    pid_ctx_t *pid = (pid_ctx_t *)arg;

    // Print all internal PID values for debugging
    debug_io_log_info("PID Internal State:\n");
    debug_io_log_info("  Gains: kp=%d, ki=%d, kd=%d\n", pid->kp, pid->ki, pid->kd);
    debug_io_log_info("  Setpoint: %d, Current Input: %d\n", pid->setpoint, encoder_position_get());
    debug_io_log_info("  Error: %d\n", pid->setpoint - (encoder_position_get()));
    debug_io_log_info("  P-term: %d\n", pid->p_error);
    debug_io_log_info("  I-term: %d\n", pid->i_error);
    debug_io_log_info("  D-term: %d\n", pid->d_error);
    debug_io_log_info("  Output: %d\n", pid->output);
}

void debug_term_pid_toggle(const char *input, void *arg)
{
    // Toggle PID enable/disable
    pid_enabled        = !pid_enabled;
    debug_motor.enable = pid_enabled;
    motor_control(&debug_motor);
    debug_io_log_info("PID: %s\n", pid_enabled ? "enabled" : "disabled");
}

void debug_term_debug_pin(const char *input, void *arg)
{
    int pin, value;
    int num_args = sscanf(input, "%d %d", &pin, &value);

    // Check if pin number is valid (1-4, but internally we use 0-3)
    if (pin < 1 || pin > 4) {
        debug_io_log_info("Invalid pin number. Must be between 1 and 4.\n");
        return;
    }

    // Convert to 0-based indexing
    pin = pin - 1;

    if (num_args == 1) {
        // Only pin number provided, toggle the pin
        debug_pin_toggle(pin);
    } else if (num_args == 2) {
        // Both pin and value provided, set the pin
        if (value != 0 && value != 1) {
            debug_io_log_info("Invalid value. Must be 0 or 1.\n");
            return;
        }
        debug_pin_set(pin, (bool)value);
    } else {
        debug_io_log_info("Usage: d <pin_number> [value]\n");
        debug_io_log_info("  pin_number: 1-4\n");
        debug_io_log_info("  value: 0 or 1 (optional, toggles if not provided)\n");
        return;
    }

    // Print the new value
    bool current_state = debug_pin_get(pin);
    debug_io_log_info("Debug pin %d: %s\n", pin + 1, current_state ? "HIGH" : "LOW");
}
