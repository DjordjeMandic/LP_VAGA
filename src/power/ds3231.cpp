#include <Arduino.h>
#include <power/ds3231.hpp>

static unsigned long ds3231_power_on_millis_ = 0;

unsigned long ds3231_get_power_on_millis()
{
    return ds3231_power_on_millis_;
}

void ds3231_power_off()
{
    pinMode(DS3231_POWER_PIN, OUTPUT);
    digitalWrite(DS3231_POWER_PIN, DS3231_POWER_PIN_STATE_OFF);
}

void ds3231_power_float()
{
    pinMode(DS3231_POWER_PIN, INPUT);
}

void ds3231_rtc_power_on()
{
    pinMode(DS3231_POWER_PIN, OUTPUT);
    digitalWrite(DS3231_POWER_PIN, DS3231_POWER_PIN_STATE_ON); 
    ds3231_power_on_millis_ = millis();
}

bool ds3231_power_delay_check()
{
    return millis() - ds3231_power_on_millis_ >= DS3231_POWER_DELAY_MS;
}

bool ds3231_powered_on()
{
    return (digitalRead(DS3231_POWER_PIN) == DS3231_POWER_PIN_STATE_ON) && ds3231_power_delay_check();
}
