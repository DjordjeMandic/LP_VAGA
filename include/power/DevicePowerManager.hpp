#pragma once

#include <Arduino.h>

/**
 * @brief A templated class to manage the power states of devices with configurable power pins.
 * 
 * This class provides methods to power on/off devices, set pins to high-impedance (float),
 * and check readiness based on a defined delay after power-on. Floating functionality can 
 * be disabled for devices that must never float.
 * 
 * @tparam PowerPin The pin used to control power to the device.
 * @tparam PowerPinStateOn The pin state (HIGH/LOW) to power on the device.
 * @tparam PowerPinStateOff The pin state (HIGH/LOW) to power off the device.
 * @tparam PowerDelayMs The delay time (in milliseconds) required after powering on the device before it can reliably operate.
 * @tparam AllowFloat A boolean indicating whether the power pin can be set to a high-impedance state (true = allowed, false = disallowed).
 */
template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat = false>
class DevicePowerManager
{
private:
    static unsigned long power_on_millis_; /**< Stores the time when the device was last powered on. */

public:
    /**
     * @brief Get the time (in milliseconds) when the device was last powered on.
     * 
     * @return The value of `millis()` when the device was powered on.
     */
    static unsigned long getPowerOnMillis();

    /**
     * @brief Power off the device by setting the power pin to its OFF state.
     */
    static void powerOff();

    /**
     * @brief Set the device power pin to a floating (high-impedance) state.
     * 
     * If floating is disabled (`AllowFloat = false`), this function will not be available
     * and attempting to call it will result in a compile-time error.
     */
    static void powerFloat();

    /**
     * @brief Power on the device by setting the power pin to its ON state.
     * 
     * The time of power-on is recorded using `millis()`.
     */
    static void powerOn();

    /**
     * @brief Check if the required delay time after powering on the device has elapsed.
     * 
     * @param[in] current_millis The current time in milliseconds. If not provided, 
     * `millis()` will be used.
     * 
     * @return `true` if the delay (defined by `PowerDelayMs`) has elapsed since 
     * the device was powered on, otherwise `false`.
     */
    static bool powerDelayCheck(unsigned long current_millis = millis());

    /**
     * @brief Check if the device is powered on and ready for use.
     * 
     * This function verifies that the power pin is in the ON state and that the 
     * required delay time (`PowerDelayMs`) has elapsed since the device 
     * was powered on.
     * 
     * @param[in] current_millis The current time in milliseconds. If not provided, 
     * `millis()` will be used.
     * 
     * @return `true` if the device is powered on and ready, otherwise `false`.
     */
    static bool poweredOn(unsigned long current_millis = millis());
};

#include <power/DevicePowerManager.tpp> // Include the template definitions
