#pragma once

#include <Arduino.h>
#include <Wire.h>

#define DS3231_POWER_PIN         4
#define DS3231_POWER_DELAY_MS    500
#define DS3231_POWER_PIN_STATE_OFF     LOW
#define DS3231_POWER_PIN_STATE_ON      HIGH

unsigned long ds3231_power_on_millis = 0;

inline void ds3231_power_off()
{
    pinMode(DS3231_POWER_PIN, OUTPUT);
    digitalWrite(DS3231_POWER_PIN, DS3231_POWER_PIN_STATE_OFF);
}

inline void ds3231_power_float()
{
    pinMode(DS3231_POWER_PIN, INPUT);
}

inline void ds3231_rtc_power_on()
{
    pinMode(DS3231_POWER_PIN, OUTPUT);
    digitalWrite(DS3231_POWER_PIN, DS3231_POWER_PIN_STATE_ON); 
    ds3231_power_on_millis = millis();
}

inline bool ds3231_power_delay_check()
{
    return millis() - ds3231_power_on_millis >= DS3231_POWER_DELAY_MS;
}

inline bool ds3231_powered_on()
{
    return (digitalRead(DS3231_POWER_PIN) == DS3231_POWER_PIN_STATE_ON) && ds3231_power_delay_check();
}
