#include <Arduino.h>
#include <power/hx711.hpp>
#include <config.hpp>

static unsigned long hx711_power_on_millis_ = 0;

unsigned long hx711_get_power_on_millis()
{
    return hx711_power_on_millis_;
}

void hx711_power_off()
{
    pinMode(HX711_POWER_PIN, OUTPUT);
    digitalWrite(HX711_POWER_PIN, HX711_POWER_PIN_STATE_OFF);
}

void hx711_power_float()
{
    pinMode(HX711_POWER_PIN, INPUT);
}

void hx711_power_on()
{
    pinMode(HX711_POWER_PIN, OUTPUT);
    digitalWrite(HX711_POWER_PIN, HX711_POWER_PIN_STATE_ON);
    hx711_power_on_millis_ = millis();
}

bool hx711_power_delay_check()
{
    return millis() - hx711_power_on_millis_ >= HX711_POWER_DELAY_MS;
}

bool hx711_powered_on()
{
    return (digitalRead(HX711_POWER_PIN) == HX711_POWER_PIN_STATE_ON) && hx711_power_delay_check();
}
