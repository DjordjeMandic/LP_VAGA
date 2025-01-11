#pragma once

#include <Arduino.h>
#include <power/manager.hpp>
#include <Config.hpp>

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
 * @brief Power manager for the DS3231 real-time clock module.
 * 
 * Manages power states (ON, OFF, floating) and ensures a stabilization delay 
 * of 500 ms after powering on. Allows floating of the power pin when not in use.
 * 
 * - **Pin**: Controlled via `DS3231_POWER_PIN`.
 * 
 * - **Power States**: ON (`DS3231_POWER_PIN_STATE_ON`), OFF (`DS3231_POWER_PIN_STATE_OFF`).
 * 
 * - **Delay**: Requires 500 ms after powering on.
 */
using DS3231PowerManager = DevicePowerManager<DS3231_POWER_PIN, DS3231_POWER_PIN_STATE_ON, DS3231_POWER_PIN_STATE_OFF, DS3231_POWER_DELAY_MS, true>;
