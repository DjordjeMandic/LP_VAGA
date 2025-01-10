#pragma once

#include <Arduino.h>

/**
 * @brief Delay time (in milliseconds) after powering on the DS3231 module before it can reliably operate.
 */
#define DS3231_POWER_DELAY_MS       500

/**
 * @brief Pin state for powering off the DS3231 module.
 */
#define DS3231_POWER_PIN_STATE_OFF  LOW

/**
 * @brief Pin state for powering on the DS3231 module.
 */
#define DS3231_POWER_PIN_STATE_ON   HIGH

/**
 * @brief Get the time (in milliseconds) when the DS3231 module was last powered on.
 * 
 * @return The value of `millis()` when the DS3231 was powered on.
 */
unsigned long ds3231_get_power_on_millis();

/**
 * @brief Power off the DS3231 module by setting the power pin to its OFF state.
 * 
 * The power pin is configured as an output, and its state is set to 
 * `DS3231_POWER_PIN_STATE_OFF`.
 */
void ds3231_power_off();

/**
 * @brief Set the DS3231 power pin to a floating (high-impedance) state.
 * 
 * The power pin is configured as an input, effectively floating it.
 */
void ds3231_power_float();

/**
 * @brief Power on the DS3231 module by setting the power pin to its ON state.
 * 
 * The power pin is configured as an output, and its state is set to 
 * `DS3231_POWER_PIN_STATE_ON`. The time of power-on is recorded using `millis()`.
 */
void ds3231_power_on();

/**
 * @brief Check if the required delay time after powering on the DS3231 module has elapsed.
 * 
 * @return `true` if the delay (defined by `DS3231_POWER_DELAY_MS`) has elapsed since 
 * the DS3231 was powered on, otherwise `false`.
 */
bool ds3231_power_delay_check();

/**
 * @brief Check if the DS3231 module is powered on and ready for use.
 * 
 * This function verifies that the power pin is in the ON state and that the 
 * required delay time (`DS3231_POWER_DELAY_MS`) has elapsed since the module 
 * was powered on.
 * 
 * @return `true` if the module is powered on and ready, otherwise `false`.
 */
bool ds3231_powered_on();
