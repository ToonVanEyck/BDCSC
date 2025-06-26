/**
 * \file peripherals.h
 *
 * Used Peripherals:
 * - TIMER 1:
 *   - CH1 Input Capture of COMP2 event (motor commutator pulses).
 *   - CH3 & CH4 PWM output for motor control.
 *   - Tick timer for the system.
 * - TIMER 3: Quadrature encoder input.
 * - COMPARATOR 2: Motor commutator pulses.
 */

#pragma once

#include "py32f0xx_bsp_clock.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_comp.h"
#include "py32f0xx_ll_dma.h"
#include "py32f0xx_ll_flash.h"
#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_tim.h"
#include "py32f0xx_ll_utils.h"

#include <stdbool.h>
#include <stdint.h>

/** Type for controlling the motor. */
typedef struct {
    bool enable;       /**< Enable motor. */
    uint16_t speed;    /**< Motor speed (0-1000). */
    bool direction_cw; /**< Motor direction: true for clockwise, false for counter-clockwise. */
} motor_t;

/**
 * @brief Initializes all peripherals.
 */
void peripherals_init(void);

/**
 * @brief Deinitializes all peripherals.
 *
 * @note This function is not implemented.
 */
void peripherals_deinit(void);

/**
 * @brief Sets a debug pin to a specific value.
 *
 * @param pin The pin number (0 to 3).
 * @param value The value to set (0 or 1).
 */
void debug_pin_set(uint8_t pin, bool value);

/**
 * @brief Toggles a debug pin.
 *
 * @param pin The pin number (0 to 3).
 */
void debug_pin_toggle(uint8_t pin);

/**
 * @brief Gets the current state of a debug pin.
 *
 * @param pin The pin number (0 to 3).
 * @return The state of the pin (true for high, false for low).
 */
bool debug_pin_get(uint8_t pin);

/**
 * @brief Get the value of the TIM1 tick count.
 *
 * This counter increments on each TIM1 update event.
 */
uint32_t tim1_tick_count_get(void);

/**
 * @brief Get the encoder position from TIM3.
 *
 * @return The current encoder position scaled to match the sensorless method.
 */
uint16_t encoder_position_get(void);

/**
 * @brief Updates the motor control based on the provided motor context.
 *
 * @param motor Pointer to the motor context containing speed and direction.
 */
void motor_control(const motor_t *motor);

/**
 * @brief Get the input capture DMA buffer status.
 *
 * @return true if buffer is ready for processing, false otherwise.
 */
bool ic_dma_ready_get(void);

/**
 * @brief Clear the input capture DMA ready flag.
 */
void ic_dma_ready_clear(void);

/**
 * @brief Get pointer to the input capture DMA buffer.
 *
 * @return Pointer to the DMA buffer containing captured timer values.
 */
const uint32_t *ic_dma_buffer_get(void);

/**
 * @brief Get the size of the COMP2 input capture DMA buffer.
 *
 * @return Size of the DMA buffer in elements.
 */
uint32_t ic_dma_buffer_size_get(void);