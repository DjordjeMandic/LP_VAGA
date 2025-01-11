#pragma once

#include <Arduino.h>
#include <power/DevicePowerManager.hpp>
#include <Config.hpp>

/**
 * @brief Delay time (in milliseconds) after powering on the HX711 module before it can reliably operate.
 */
#define HX711_POWER_DELAY_MS        500

/**
 * @brief Pin state for powering off the HX711 module.
 */
#define HX711_POWER_PIN_STATE_OFF   LOW

/**
 * @brief Pin state for powering on the HX711 module.
 */
#define HX711_POWER_PIN_STATE_ON    HIGH

/**
 * @brief Power manager for the HX711 load cell amplifier module.
 * 
 * Manages power states (ON, OFF, floating) and ensures a stabilization delay 
 * of 500 ms after powering on. Allows floating of the power pin when not in use.
 * 
 * - **Pin**: Controlled via `HX711_POWER_PIN`.
 * 
 * - **Power States**: ON (`HX711_POWER_PIN_STATE_ON`), OFF (`HX711_POWER_PIN_STATE_OFF`).
 * 
 * - **Delay**: Requires 500 ms after powering on.
 */
using HX711PowerManager = DevicePowerManager<HX711_POWER_PIN, HX711_POWER_PIN_STATE_ON, HX711_POWER_PIN_STATE_OFF, HX711_POWER_DELAY_MS, true>;
