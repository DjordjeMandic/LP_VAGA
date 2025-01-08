#pragma once

#include <Arduino.h>

#define HX711_POWER_PIN             8
#define HX711_POWER_DELAY_MS        500
#define HX711_POWER_PIN_STATE_OFF   LOW
#define HX711_POWER_PIN_STATE_ON    HIGH

unsigned long hx711_power_on_millis = 0;

/*
 * Turns the HX711 power off by setting the corresponding pin to LOW.
 */
inline void hx711_power_off()
{
    pinMode(HX711_POWER_PIN, OUTPUT);
    digitalWrite(HX711_POWER_PIN, HX711_POWER_PIN_STATE_OFF);
}

inline void hx711_power_float()
{
    pinMode(HX711_POWER_PIN, INPUT);
}

/*
 * Turns the HX711 power on by setting the corresponding pin to HIGH 
 * and records the current time using millis().
 */
inline void hx711_power_on()
{
    pinMode(HX711_POWER_PIN, OUTPUT);
    digitalWrite(HX711_POWER_PIN, HX711_POWER_PIN_STATE_ON);
    hx711_power_on_millis = millis();
}

/*
 * Checks whether the delay period after turning the HX711 on has passed.
 * Returns true if the delay has elapsed, false otherwise.
 */
inline bool hx711_power_delay_check()
{
    return millis() - hx711_power_on_millis >= HX711_POWER_DELAY_MS;
}

inline bool hx711_powered_on()
{
    return (digitalRead(HX711_POWER_PIN) == HX711_POWER_PIN_STATE_ON) && hx711_power_delay_check();
}
