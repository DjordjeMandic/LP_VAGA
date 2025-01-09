#include <Arduino.h>
#include <power/sim800.hpp>
#include <config.hpp>

static unsigned long sim800_power_on_millis_ = 0;

unsigned long sim800_get_power_on_millis()
{
    return sim800_power_on_millis_;
}

void sim800_power_off()
{
    pinMode(SIM800_POWER_PIN, OUTPUT);
    digitalWrite(SIM800_POWER_PIN, SIM800_POWER_PIN_STATE_OFF);
}

void sim800_power_on()
{
    pinMode(SIM800_POWER_PIN, OUTPUT);
    digitalWrite(SIM800_POWER_PIN, SIM800_POWER_PIN_STATE_ON);
    sim800_power_on_millis_ = millis();
}

bool sim800_power_delay_check()
{
    return millis() - sim800_power_on_millis_ >= SIM800_POWER_DELAY_MS;
}

bool sim800_powered_on()
{
    return (digitalRead(SIM800_POWER_PIN) == SIM800_POWER_PIN_STATE_ON) && sim800_power_delay_check();
}
