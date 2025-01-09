#include <Arduino.h>
#include <avr/power.h>
#include <power/uart.hpp>

void uart_power_on()
{
    power_usart0_enable();
}

void uart_power_off()
{
    power_usart0_disable();
    pinMode(0, INPUT);
    pinMode(1, INPUT);
}