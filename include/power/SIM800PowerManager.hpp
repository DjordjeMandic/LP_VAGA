#pragma once

#include <power/DevicePowerManager.hpp>
#include <Config.hpp>

/**
 * @brief Delay time (in milliseconds) after powering on the SIM800 GSM module before it can reliably provide readings.
 */
#define SIM800_POWER_DELAY_MS        500

/**
 * @brief Pin state for powering off the SIM800 GSM module.
 */
#define SIM800_POWER_PIN_STATE_OFF   LOW

/**
 * @brief Pin state for powering on the SIM800 GSM module.
 */
#define SIM800_POWER_PIN_STATE_ON    HIGH

static_assert(SIM800_POWER_DELAY_MS >= 500, "SIM800_POWER_DELAY_MS must be at least 500 ms.");

/**
 * @brief Power manager for the SIM800 GSM module.
 * 
 * Manages power states (ON, OFF) and ensures a stabilization delay 
 * of 500 ms after powering on. Does not allow floating of the power pin.
 * 
 * - **Pin**: Controlled via `SIM800_POWER_PIN`.
 * 
 * - **Power States**: ON (`SIM800_POWER_PIN_STATE_ON`), OFF (`SIM800_POWER_PIN_STATE_OFF`).
 * 
 * - **Delay**: Requires 500 ms after powering on.
 */
using SIM800PowerManager = DevicePowerManager<SIM800_POWER_PIN, SIM800_POWER_PIN_STATE_ON, SIM800_POWER_PIN_STATE_OFF, SIM800_POWER_DELAY_MS, false>;
