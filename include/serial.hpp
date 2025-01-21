#pragma once

#include <Arduino.h>
#include <Config.hpp>

#define serial_flush() { if (serial_is_enabled()) { Serial.flush(); } }
#define serial_printf(format, ...) \
    { if (serial_is_enabled()) { Serial.printf(format, ##__VA_ARGS__); } }


void serial_begin(unsigned long baudrate = SERIAL_BAUD);
void serial_end();

bool serial_is_enabled();