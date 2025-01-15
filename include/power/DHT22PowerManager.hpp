#pragma once

#include <Arduino.h>
#include <power/DevicePowerManager.hpp>
#include <Config.hpp>

/**
 * @brief Delay time (in milliseconds) after powering on the DHT22 sensor before it can reliably provide readings.
 */
#define DHT22_POWER_DELAY_MS       2050

/**
 * @brief Pin state for powering off the DHT22 sensor.
 */
#define DHT22_POWER_PIN_STATE_OFF  LOW

/**
 * @brief Pin state for powering on the DHT22 sensor.
 */
#define DHT22_POWER_PIN_STATE_ON   HIGH

static_assert(DHT22_POWER_DELAY_MS >= 2050, "DHT22_POWER_DELAY_MS must be at least 2050 ms. (50ms startup time)");

/**
 * @brief Power manager for the DHT22 temperature and humidity sensor.
 * 
 * Manages power states (ON, OFF, floating) and ensures a stabilization delay 
 * of 3000 ms after powering on. Allows floating of the power pin when not in use.
 * 
 * - **Pin**: Controlled via `DHT22_POWER_PIN`.
 * 
 * - **Power States**: ON (`DHT22_POWER_PIN_STATE_ON`), OFF (`DHT22_POWER_PIN_STATE_OFF`).
 * 
 * - **Delay**: Requires 2050 ms after powering on.
 */
using DHT22PowerManager = DevicePowerManager<DHT22_POWER_PIN, DHT22_POWER_PIN_STATE_ON, DHT22_POWER_PIN_STATE_OFF, DHT22_POWER_DELAY_MS, true>;