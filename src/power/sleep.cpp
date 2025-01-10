#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <power/sleep.hpp>

void sleep_idle_timeout_millis(unsigned long sleep_time_ms)
{
    unsigned long start = millis();

    /* Sleep until the specified sleep time is reached */
    sleep_while(SLEEP_MODE_IDLE, (millis() - start) < sleep_time_ms);
}