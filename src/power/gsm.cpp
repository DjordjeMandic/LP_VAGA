#include <Arduino.h>
#include <power/gsm.hpp>
#include <Config.h>

static unsigned long gsm_power_on_millis_ = 0;

unsigned long gsm_get_power_on_millis()
{
    return gsm_power_on_millis_;
}

void gsm_power_off()
{
    pinMode(GSM_POWER_PIN, OUTPUT);
    digitalWrite(GSM_POWER_PIN, GSM_POWER_PIN_STATE_OFF);
}

void gsm_power_on()
{
    pinMode(GSM_POWER_PIN, OUTPUT);
    digitalWrite(GSM_POWER_PIN, GSM_POWER_PIN_STATE_ON);
    gsm_power_on_millis_ = millis();
}

bool gsm_power_delay_check()
{
    return millis() - gsm_power_on_millis_ >= GSM_POWER_DELAY_MS;
}

bool gsm_powered_on()
{
    return (digitalRead(GSM_POWER_PIN) == GSM_POWER_PIN_STATE_ON) && gsm_power_delay_check();
}
