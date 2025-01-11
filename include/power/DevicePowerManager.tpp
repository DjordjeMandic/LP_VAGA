#pragma once

#include <Arduino.h>
#include <power/DevicePowerManager.hpp>

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::getPowerOnMillis()
{
    return power_on_millis_;
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOff()
{
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, PowerPinStateOff);
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOn()
{
    if (PowerPinStateOn == HIGH)
    {
        pinMode(PowerPin, INPUT_PULLUP);
        delay(1);
    }
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, PowerPinStateOn);
    power_on_millis_ = millis();
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerDelayCheck()
{
    return millis() - power_on_millis_ >= PowerDelayMs;
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::poweredOn()
{
    return (digitalRead(PowerPin) == PowerPinStateOn) && powerDelayCheck();
}

// Only provide the powerFloat() function if AllowFloat is true
template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerFloat()
{
    static_assert(AllowFloat, "Floating is not allowed for this device.");
    pinMode(PowerPin, INPUT);
}
