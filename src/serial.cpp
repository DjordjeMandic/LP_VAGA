#include <Arduino.h>
#include <Serial.hpp>
#include <power/uart.hpp>

static bool serial_print_enabled_ = false;

void serial_begin(unsigned long baudrate)
{
    uart_power_on();
    Serial.begin(baudrate);
    serial_print_enabled_ = true;
}

void serial_end()
{
    if (serial_print_enabled_)
    {
        Serial.flush();
    }

    serial_print_enabled_ = false;
    Serial.end();
    uart_power_off();
}

bool serial_is_enabled()
{
    return serial_print_enabled_;
}