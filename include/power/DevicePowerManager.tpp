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
    return this->power_on_millis_;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOff()
{
    pinMode(this->PowerPin, OUTPUT);
    digitalWrite(this->PowerPin, this->PowerPinStateOff);
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerOn()
{
    if (this->AllowFloat && (this->PowerPinStateOn == HIGH))
    {
        /* Precharge the power pin */
        pinMode(this->PowerPin, INPUT_PULLUP);
        sleep_idle_timeout_millis(1);
    }
    pinMode(this->PowerPin, OUTPUT);
    digitalWrite(this->PowerPin, this->PowerPinStateOn);
    this->power_on_millis_ = millis();
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerDelayCheck(unsigned long current_millis)
{
    return current_millis - this->power_on_millis_ >= this->PowerDelayMs;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::poweredOn(unsigned long current_millis)
{
    return (this->powerEnabled() && this->powerDelayCheck(current_millis));
}

// Only provide the powerFloat() function if AllowFloat is true
template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
void DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerFloat()
{
    static_assert(this->AllowFloat, "Floating is not allowed for this device.");
    pinMode(this->PowerPin, INPUT);
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
bool DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::powerEnabled()
{
    return digitalRead(this->PowerPin) == this->PowerPinStateOn;
}

template <uint8_t PowerPin, bool PowerPinStateOn, bool PowerPinStateOff, unsigned long PowerDelayMs, bool AllowFloat>
unsigned long DevicePowerManager<PowerPin, PowerPinStateOn, PowerPinStateOff, PowerDelayMs, AllowFloat>::requiredPowerOnDelay(unsigned long current_millis)
{
    unsigned long elapsed_time = current_millis - this->power_on_millis_;
    return (this->elapsed_time >= this->PowerDelayMs) ? 0 : (this->PowerDelayMs - this->elapsed_time);
}