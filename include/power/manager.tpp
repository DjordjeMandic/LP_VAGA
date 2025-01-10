#pragma once

#include <Arduino.h>
#include <power/manager.hpp>

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::get_power_on_millis()
{
    return power_on_millis_;
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::power_off()
{
    pinMode(PowerPin, OUTPUT);
    digitalWrite(PowerPin, PowerPinStateOff);
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::power_on()
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
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::power_delay_check()
{
    return millis() - power_on_millis_ >= PowerDelayMs;
}

template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powered_on()
{
    return (digitalRead(PowerPin) == PowerPinStateOn) && power_delay_check();
}

// Only provide the power_float() function if AllowFloat is true
template <uint8_t PowerPin, uint8_t PowerPinStateOn, uint8_t PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::power_float()
{
    static_assert(AllowFloat, "Floating is not allowed for this device.");
    pinMode(PowerPin, INPUT);
}
