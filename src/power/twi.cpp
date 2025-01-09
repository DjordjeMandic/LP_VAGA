#include <Arduino.h>
#include <Wire.h>
#include <avr/power.h>
#include <power/twi.hpp>

void twi_power_on()
{
    power_twi_enable();
}

void twi_power_off()
{
    Wire.end();
    power_twi_disable();
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
}