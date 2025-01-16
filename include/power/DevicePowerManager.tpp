#pragma once

#include <Arduino.h>
#include <power/DevicePowerManager.hpp>
#include <power/sleep.hpp>

// Define the static variable for the template
template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::power_on_millis_ = 0;

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::getPowerOnMillis()
{
    return power_on_millis_;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOff()
{
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, PowerPinStateOff);
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOn()
{
    if (AllowFloat && (PowerPinStateOn == HIGH))
    {
        /* Precharge the power pin */
        pinMode(PowerPin, INPUT_PULLUP);
        sleep_idle_timeout_millis(1);
    }
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, PowerPinStateOn);
    power_on_millis_ = millis();
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerDelayCheck(unsigned long current_millis)
{
    return current_millis - power_on_millis_ >= PowerDelayMs;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::poweredOn(unsigned long current_millis)
{
    return (powerEnabled() && powerDelayCheck(current_millis));
}

// Only provide the powerFloat() function if AllowFloat is true
template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerFloat()
{
    static_assert(AllowFloat, "Floating is not allowed for this device.");
    pinMode(PowerPin, INPUT);
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerEnabled()
{
    return digitalRead(PowerPin) == PowerPinStateOn;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::requiredPowerOnDelay(unsigned long current_millis)
{
    unsigned long elapsed_time = current_millis - power_on_millis_;
    return (elapsed_time >= PowerDelayMs) ? 0 : (PowerDelayMs - elapsed_time);
}