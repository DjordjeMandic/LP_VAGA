#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <power/sleep.hpp>

void sleep_timeout_millis(uint8_t mode, unsigned long sleep_time_ms)
{
    unsigned long start = millis();
    sleep_while(mode, (millis() - start) < sleep_time_ms);
}