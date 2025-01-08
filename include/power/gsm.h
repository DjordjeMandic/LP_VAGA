#pragma once

#include <Arduino.h>

#define GSM_POWER_PIN         5
#define GSM_POWER_DELAY_MS    500
#define GSM_POWER_PIN_STATE_OFF     LOW
#define GSM_POWER_PIN_STATE_ON      HIGH

unsigned long gsm_power_on_millis = 0;

inline void gsm_power_off()
{
    pinMode(GSM_POWER_PIN, OUTPUT);
    digitalWrite(GSM_POWER_PIN, GSM_POWER_PIN_STATE_OFF);
}

inline void gsm_power_on()
{
    pinMode(GSM_POWER_PIN, OUTPUT);
    digitalWrite(GSM_POWER_PIN, GSM_POWER_PIN_STATE_ON);
    gsm_power_on_millis = millis();
}

inline bool gsm_power_delay_check()
{
    return millis() - gsm_power_on_millis >= GSM_POWER_DELAY_MS;
}

inline bool gsm_powered_on()
{
    return (digitalRead(GSM_POWER_PIN) == GSM_POWER_PIN_STATE_ON) && gsm_power_delay_check();
}
