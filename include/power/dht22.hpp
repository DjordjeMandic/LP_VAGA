#pragma once

#include <Arduino.h>

/**
 * @brief Delay time (in milliseconds) after powering on the DHT22 sensor before it can reliably provide readings.
 */
#define DHT22_POWER_DELAY_MS       3000

/**
 * @brief Pin state for powering off the DHT22 sensor.
 */
#define DHT22_POWER_PIN_STATE_OFF  LOW

/**
 * @brief Pin state for powering on the DHT22 sensor.
 */
#define DHT22_POWER_PIN_STATE_ON   HIGH

/**
 * @brief Get the time (in milliseconds) when the DHT22 was last powered on.
 * 
 * @return The value of `millis()` when the DHT22 was powered on.
 */
unsigned long dht22_get_power_on_millis();

/**
 * @brief Power off the DHT22 sensor by setting the power pin to its OFF state.
 * 
 * The power pin is configured as an output, and its state is set to 
 * `DHT22_POWER_PIN_STATE_OFF`.
 */
void dht22_power_off();

/**
 * @brief Set the DHT22 power pin to a floating (high-impedance) state.
 * 
 * The power pin is configured as an input, effectively floating it.
 */
void dht22_power_float();

/**
 * @brief Power on the DHT22 sensor by setting the power pin to its ON state.
 * 
 * The power pin is configured as an output, and its state is set to 
 * `DHT22_POWER_PIN_STATE_ON`. The time of power-on is recorded using `millis()`.
 */
void dht22_power_on();

/**
 * @brief Check if the required delay time after powering on the DHT22 sensor has elapsed.
 * 
 * @return `true` if the delay (defined by `DHT22_POWER_DELAY_MS`) has elapsed since 
 * the DHT22 was powered on, otherwise `false`.
 */
bool dht22_power_delay_check();

/**
 * @brief Check if the DHT22 sensor is powered on and ready for use.
 * 
 * This function verifies that the power pin is in the ON state and that the 
 * required delay time (`DHT22_POWER_DELAY_MS`) has elapsed since the sensor 
 * was powered on.
 * 
 * @return `true` if the sensor is powered on and ready, otherwise `false`.
 */
bool dht22_powered_on();